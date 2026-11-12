// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cuda_runtime_api.h>
#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "curobo_robot_runtime/kinematics.h"
#include "curobo_robot_runtime/motion_planner.h"
#include "curobo_robot_runtime/mpc.h"
#include "curobo_robot_runtime/scene.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace curobo_ros_cpp {
namespace {

void check_cuda(cudaError_t status, const char* where) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(where) + ": " + cudaGetErrorString(status));
  }
}

double to_seconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

std::vector<std::string> split_names(const std::string& text) {
  std::vector<std::string> names;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::string entry = text.substr(
        start, comma == std::string::npos ? std::string::npos : comma - start);
    if (!entry.empty()) names.push_back(entry);
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return names;
}

}  // namespace

/**
 * Experimental open-loop MPC horizon node.
 *
 * One feasible horizon is computed for a Cartesian goal and streamed to a
 * ForwardCommandController. It does NOT close the loop yet: accept() is called
 * when streaming starts, and a new goal triggers a new horizon. This is the
 * scaffold for the full 30-33 Hz step/accept/validate loop.
 */
class MpcNode : public rclcpp::Node {
 public:
  MpcNode() : rclcpp::Node("curobo_mpc") {
    artifact_dir_ = this->declare_parameter<std::string>("artifact_dir", "");
    if (artifact_dir_.empty()) {
      throw std::runtime_error("artifact_dir is required");
    }
    const std::string joint_states_topic =
        this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const std::string goal_topic =
        this->declare_parameter<std::string>("goal_topic", "/curobo_mpc/goal");
    command_topic_ = this->declare_parameter<std::string>(
        "command_topic", "/position_forward_controller/commands");
    mpc_rate_ = this->declare_parameter<double>("mpc_rate", 30.0);
    command_rate_ = this->declare_parameter<double>("command_rate", 100.0);
    const double trajectory_dt = this->declare_parameter<double>(
        "trajectory_dt", 0.03);
    const auto controller_joints =
        this->declare_parameter<std::vector<std::string>>(
            "controller_joints", std::vector<std::string>{});
    const std::string name_map =
        this->declare_parameter<std::string>("joint_name_map", "");

    std::size_t start = 0;
    while (start < name_map.size()) {
      const std::size_t comma = name_map.find(',', start);
      const std::string entry = name_map.substr(
          start, comma == std::string::npos ? std::string::npos : comma - start);
      const std::size_t separator = entry.find(':');
      if (separator != std::string::npos) {
        name_map_[entry.substr(0, separator)] = entry.substr(separator + 1);
      }
      if (comma == std::string::npos) break;
      start = comma + 1;
    }

    kinematics_ = std::make_unique<curobo::robot::Kinematics>(artifact_dir_);

    curobo::robot::SceneConfig scene_config;
    scene_config.max_cuboids = 8;
    scene_config.max_capsules = 8;
    scene_config.max_cylinders = 8;
    scene_ = std::make_unique<curobo::robot::Scene>(scene_config);

    curobo::robot::MotionPlanningConfig planner_config;
    planner_config.use_cuda_graph = true;
    planner_ = std::make_unique<curobo::robot::MotionPlanner>(
        artifact_dir_, planner_config, scene_.get());

    curobo::robot::MpcConfig mpc_config;
    mpc_config.trajectory_dt = static_cast<float>(trajectory_dt);
    mpc_ = std::make_unique<curobo::robot::MpcController>(*planner_, mpc_config);

    const auto robot = planner_->robot_info();
    dof_ = robot.num_dof;
    tool_count_ = robot.num_tool_frames;
    if (dof_ <= 0) {
      throw std::runtime_error("artifact has no active joints");
    }
    std::vector<std::string> artifact_names;
    artifact_names.reserve(static_cast<std::size_t>(dof_));
    for (int i = 0; i < dof_; ++i) {
      artifact_names.push_back(kinematics_->joint_name(i));
    }
    controller_names_ = controller_joints.empty()
                            ? artifact_names
                            : controller_joints;
    if (controller_names_.size() != static_cast<std::size_t>(dof_)) {
      throw std::runtime_error("controller_joints size must match artifact DOF");
    }
    controller_source_.resize(controller_names_.size());
    for (std::size_t i = 0; i < controller_names_.size(); ++i) {
      const auto it = std::find(artifact_names.begin(), artifact_names.end(),
                                controller_names_[i]);
      if (it == artifact_names.end()) {
        throw std::runtime_error("controller joint not in artifact: " + controller_names_[i]);
      }
      controller_source_[i] = static_cast<std::size_t>(
          std::distance(artifact_names.begin(), it));
    }

    const auto shape = mpc_->info();
    steps_ = shape.trajectory_steps;
    knots_ = shape.num_knots;
    if (steps_ <= 0 || knots_ <= 0) {
      throw std::runtime_error("MPC artifact has no trajectory layout");
    }

    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_q_), dof_ * sizeof(float)), "malloc q");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_v_), dof_ * sizeof(float)), "malloc v");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_a_), dof_ * sizeof(float)), "malloc a");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_j_), dof_ * sizeof(float)), "malloc jerk");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_goal_pos_), tool_count_ * 3 * sizeof(float)),
               "malloc goal pos");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_goal_quat_), tool_count_ * 4 * sizeof(float)),
               "malloc goal quat");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_seed_), knots_ * dof_ * sizeof(float)),
               "malloc seed");

    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        command_topic_, 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/status", rclcpp::QoS(1).transient_local());
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic, 10,
        std::bind(&MpcNode::joint_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, std::bind(&MpcNode::goal_callback, this, std::placeholders::_1));
    cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/cancel", std::bind(&MpcNode::cancel_callback, this, std::placeholders::_1,
                              std::placeholders::_2));

    solve_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(mpc_rate_, 1.0)),
        std::bind(&MpcNode::solve_callback, this));
    stream_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(command_rate_, 1.0)),
        std::bind(&MpcNode::stream_callback, this));

    set_status("idle");
    RCLCPP_INFO(this->get_logger(),
                "MPC skeleton ready: artifact=%s dof=%d knots=%d steps=%d mpc=%.1f Hz "
                "command=%.1f Hz topic=%s",
                artifact_dir_.c_str(), dof_, knots_, steps_, mpc_rate_, command_rate_,
                command_topic_.c_str());
  }

  ~MpcNode() override {
    cudaFree(d_q_);
    cudaFree(d_v_);
    cudaFree(d_a_);
    cudaFree(d_j_);
    cudaFree(d_goal_pos_);
    cudaFree(d_goal_quat_);
    cudaFree(d_seed_);
  }

 private:
  void set_status(const std::string& status) {
    if (status_ == status) return;
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  bool current_state(std::vector<float>& q) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_state_) return false;
    q.resize(static_cast<std::size_t>(dof_));
    for (int i = 0; i < dof_; ++i) {
      q[i] = state_.count(artifact_joint_name(i)) ? state_.at(artifact_joint_name(i)) : 0.0F;
    }
    return true;
  }

  std::string artifact_joint_name(int index) const {
    return kinematics_->joint_name(index);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.clear();
    for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
      const auto mapped = name_map_.find(message->name[i]);
      const std::string& name = mapped == name_map_.end() ? message->name[i] : mapped->second;
      state_[name] = static_cast<float>(message->position[i]);
    }
    has_state_ = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    std::vector<float> q;
    if (!current_state(q)) {
      set_status("waiting for /joint_states");
      return;
    }
    const auto& orientation = message->pose.orientation;
    const double norm = std::sqrt(orientation.x * orientation.x +
                                  orientation.y * orientation.y +
                                  orientation.z * orientation.z +
                                  orientation.w * orientation.w);
    if (!std::isfinite(norm) || norm < 1e-9) {
      set_status("invalid goal quaternion");
      return;
    }

    std::vector<float> positions(static_cast<std::size_t>(tool_count_) * 3);
    std::vector<float> quaternions(static_cast<std::size_t>(tool_count_) * 4);
    for (int tool = 0; tool < tool_count_; ++tool) {
      positions[static_cast<std::size_t>(tool) * 3 + 0] =
          static_cast<float>(message->pose.position.x);
      positions[static_cast<std::size_t>(tool) * 3 + 1] =
          static_cast<float>(message->pose.position.y);
      positions[static_cast<std::size_t>(tool) * 3 + 2] =
          static_cast<float>(message->pose.position.z);
      quaternions[static_cast<std::size_t>(tool) * 4 + 0] =
          static_cast<float>(orientation.w / norm);
      quaternions[static_cast<std::size_t>(tool) * 4 + 1] =
          static_cast<float>(orientation.x / norm);
      quaternions[static_cast<std::size_t>(tool) * 4 + 2] =
          static_cast<float>(orientation.y / norm);
      quaternions[static_cast<std::size_t>(tool) * 4 + 3] =
          static_cast<float>(orientation.z / norm);
    }

    std::vector<float> seed(static_cast<std::size_t>(knots_) * dof_);
    for (int knot = 0; knot < knots_; ++knot) {
      std::copy(q.begin(), q.end(), seed.begin() + static_cast<std::size_t>(knot) * dof_);
    }
    check_cuda(cudaMemcpy(d_goal_pos_, positions.data(),
                          positions.size() * sizeof(float), cudaMemcpyHostToDevice),
               "copy goal pos");
    check_cuda(cudaMemcpy(d_goal_quat_, quaternions.data(),
                          quaternions.size() * sizeof(float), cudaMemcpyHostToDevice),
               "copy goal quat");
    check_cuda(cudaMemcpy(d_seed_, seed.data(), seed.size() * sizeof(float),
                          cudaMemcpyHostToDevice),
               "copy seed");

    curobo::robot::MpcCartesianConfig objective;
    objective.constrain_path = true;
    mpc_->reset_cartesian(d_goal_pos_, d_goal_quat_, d_seed_, objective);
    pending_goal_ = true;
    solved_ = false;
    set_status("goal received");
  }

  void solve_callback() {
    if (!pending_goal_ || solved_) return;
    std::vector<float> q;
    if (!current_state(q)) return;

    std::vector<float> zero(static_cast<std::size_t>(dof_), 0.0F);
    check_cuda(cudaMemcpy(d_q_, q.data(), q.size() * sizeof(float), cudaMemcpyHostToDevice),
               "copy q");
    check_cuda(cudaMemcpy(d_v_, zero.data(), zero.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "copy v");
    check_cuda(cudaMemcpy(d_a_, zero.data(), zero.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "copy a");
    check_cuda(cudaMemcpy(d_j_, zero.data(), zero.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "copy jerk");

    const auto begin = std::chrono::steady_clock::now();
    const auto result = mpc_->step(d_q_, d_v_, d_a_, d_j_, 0);
    const double solve_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - begin)
                                .count();
    RCLCPP_INFO(this->get_logger(), "MPC step feasible=%s solve_ms=%.2f",
                result.feasible ? "true" : "false", solve_ms);
    if (!result.feasible) {
      set_status("infeasible");
      return;
    }

    horizon_.resize(static_cast<std::size_t>(steps_) * dof_);
    check_cuda(cudaMemcpy(horizon_.data(), result.position,
                          horizon_.size() * sizeof(float), cudaMemcpyDeviceToHost),
               "copy MPC horizon");
    horizon_dt_ = result.trajectory_dt;
    stream_start_ = std::chrono::steady_clock::now();
    streaming_ = true;
    solved_ = true;
    mpc_->accept();
    set_status("streaming");
    stream_callback();
  }

  void stream_callback() {
    if (!streaming_) return;
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - stream_start_)
                               .count();
    const double duration = horizon_dt_ * static_cast<double>(steps_ - 1);
    if (elapsed >= duration) {
      publish_sample(static_cast<std::size_t>(steps_ - 1), 0.0);
      streaming_ = false;
      set_status("completed");
      RCLCPP_INFO(this->get_logger(), "MPC horizon stream completed");
      return;
    }
    const std::size_t index = std::min(
        static_cast<std::size_t>(elapsed / horizon_dt_),
        static_cast<std::size_t>(steps_ - 1));
    const double base = static_cast<double>(index) * horizon_dt_;
    const double ratio = horizon_dt_ > 0.0 ? (elapsed - base) / horizon_dt_ : 0.0;
    publish_sample(index, std::clamp(ratio, 0.0, 1.0));
  }

  void publish_sample(std::size_t index, double ratio) {
    if (horizon_.empty()) return;
    std_msgs::msg::Float64MultiArray message;
    message.data.resize(controller_names_.size());
    const std::size_t next = std::min(index + 1, static_cast<std::size_t>(steps_ - 1));
    for (std::size_t i = 0; i < controller_source_.size(); ++i) {
      const std::size_t source = controller_source_[i];
      const double first = horizon_[index * dof_ + source];
      const double second = horizon_[next * dof_ + source];
      message.data[i] = static_cast<float>(first + ratio * (second - first));
    }
    command_pub_->publish(message);
  }

  void cancel_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    streaming_ = false;
    pending_goal_ = false;
    response->success = true;
    response->message = "MPC stream canceled";
    set_status("canceled");
  }

  std::string artifact_dir_;
  std::string command_topic_;
  double mpc_rate_ = 30.0;
  double command_rate_ = 100.0;
  int dof_ = 0;
  int tool_count_ = 0;
  int steps_ = 0;
  int knots_ = 0;

  std::unique_ptr<curobo::robot::Kinematics> kinematics_;
  std::unique_ptr<curobo::robot::Scene> scene_;
  std::unique_ptr<curobo::robot::MotionPlanner> planner_;
  std::unique_ptr<curobo::robot::MpcController> mpc_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr solve_timer_;
  rclcpp::TimerBase::SharedPtr stream_timer_;

  std::vector<std::string> controller_names_;
  std::vector<std::size_t> controller_source_;
  std::unordered_map<std::string, std::string> name_map_;

  std::mutex mutex_;
  std::unordered_map<std::string, float> state_;
  bool has_state_ = false;

  bool pending_goal_ = false;
  bool solved_ = false;
  bool streaming_ = false;
  std::vector<float> horizon_;
  float horizon_dt_ = 0.0F;
  std::chrono::steady_clock::time_point stream_start_{};
  std::string status_ = "idle";

  float* d_q_ = nullptr;
  float* d_v_ = nullptr;
  float* d_a_ = nullptr;
  float* d_j_ = nullptr;
  float* d_goal_pos_ = nullptr;
  float* d_goal_quat_ = nullptr;
  float* d_seed_ = nullptr;
};

}  // namespace curobo_ros_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<curobo_ros_cpp::MpcNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exception) {
    std::cerr << "mpc_node failed: " << exception.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
