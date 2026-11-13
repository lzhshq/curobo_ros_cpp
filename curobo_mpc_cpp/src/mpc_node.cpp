// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cuda_runtime_api.h>
#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "curobo_execution_cpp/execution_adapter.hpp"
#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_execution_cpp/jtc_adapter.hpp"
#include "curobo_execution_cpp/topic_adapter.hpp"
#include "curobo_robot_runtime/kinematics.h"
#include "curobo_robot_runtime/motion_planner.h"
#include "curobo_robot_runtime/mpc.h"
#include "curobo_robot_runtime/scene.h"
#include "curobo_planner_interfaces/msg/obstacle_update.hpp"
#include "curobo_msgs/action/send_trajectory.hpp"
#include "curobo_msgs/srv/get_planners.hpp"
#include "curobo_msgs/srv/set_planner.hpp"
#include "curobo_msgs/srv/trajectory_generation.hpp"
#include "curobo_msgs/msg/mpc_costs.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/convert.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
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

void set_duration(builtin_interfaces::msg::Duration& duration, double seconds) {
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9);
  if (duration.nanosec >= 1000000000U) {
    ++duration.sec;
    duration.nanosec = 0;
  }
}

}  // namespace

/**
 * Producer/consumer MPC node.
 *
 * Producer thread: step() at control_rate, validates the horizon, and stores the
 * latest proposal. It never blocks on command publishing.
 *
 * Consumer timers:
 *  - accept timer (command_interval): accept the latest validated proposal and
 *    make it the active command horizon;
 *  - JTC adapter: sends the accepted segment.
 *
 * This mirrors curobo_ros: solve continuously, send at a fixed command cadence.
 * It is still a scaffold: admission is only validation + freshness, not a real
 * controller ACK/brake check.
 */
class MpcNode : public rclcpp::Node {
 public:
  using SendTrajectory = curobo_msgs::action::SendTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SendTrajectory>;
  MpcNode() : rclcpp::Node("curobo_mpc") {
    artifact_dir_ = this->declare_parameter<std::string>("artifact_dir", "");
    if (artifact_dir_.empty()) throw std::runtime_error("artifact_dir is required");
    const std::string joint_states_topic =
        this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const std::string goal_topic =
        this->declare_parameter<std::string>("goal_topic", "/curobo_mpc/goal");
    command_topic_ = this->declare_parameter<std::string>(
        "command_topic", "/position_forward_controller/commands");
    control_rate_ = this->declare_parameter<double>("control_rate", 30.0);
    command_interval_ = this->declare_parameter<double>("command_interval", 0.48);
    adapter_name_ = this->declare_parameter<std::string>("adapter", "jtc");
    action_ = this->declare_parameter<std::string>(
        "action", "/position_joint_trajectory_controller/follow_joint_trajectory");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.02);
    goal_time_tolerance_ = this->declare_parameter<double>("goal_time_tolerance", 2.0);
    state_timeout_ = this->declare_parameter<double>("state_timeout", 0.2);
    max_tracking_error_ = this->declare_parameter<double>("max_tracking_error", 0.05);
    brake_check_ = this->declare_parameter<bool>("brake_check", true);
    brake_deceleration_ = this->declare_parameter<double>("brake_deceleration", 2.0);
    const std::string fault_topic = this->declare_parameter<std::string>(
        "fault_topic", "/curobo_mpc/fault");
    world_topic_ = this->declare_parameter<std::string>(
        "world_topic", "/curobo_world/obstacle_update");
    live_goal_topic_ = this->declare_parameter<std::string>(
        "live_goal_topic", "/curobo_mpc/live_goal");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "panda_link0");
    tool_frame_ = this->declare_parameter<std::string>("tool_frame", "panda_hand");
    bat_length_ = this->declare_parameter<double>("bat_length", 0.4);
    bat_radius_ = this->declare_parameter<double>("bat_radius", 0.03);
    bat_initial_x_ = this->declare_parameter<double>("bat_initial_x", 0.45);
    bat_initial_y_ = this->declare_parameter<double>("bat_initial_y", 0.0);
    bat_initial_z_ = this->declare_parameter<double>("bat_initial_z", 0.55);
    planner_distance_threshold_ = this->declare_parameter<double>(
        "planner_distance_threshold", 0.15);
    far_replan_period_ = this->declare_parameter<double>(
        "far_replan_period", 1.0);
    far_replan_min_move_ = this->declare_parameter<double>(
        "far_replan_min_move", 0.01);
    const double trajectory_dt =
        this->declare_parameter<double>("trajectory_dt", 0.03);
    command_steps_ = this->declare_parameter<int>("command_steps", 16);
    use_measured_state_ = this->declare_parameter<bool>("use_measured_state", false);
    if (command_steps_ < 1) command_steps_ = 1;
    const auto controller_joints = this->declare_parameter<std::vector<std::string>>(
        "controller_joints", std::vector<std::string>{});
    const std::string name_map =
        this->declare_parameter<std::string>("joint_name_map", "");
    parse_name_map(name_map);

    kinematics_ = std::make_unique<curobo::robot::Kinematics>(artifact_dir_);
    curobo_ros_cpp::PlannerConfig world_config;
    world_config.artifact_dir = artifact_dir_;
    world_config.max_cuboids = 8;
    world_config.max_capsules = 8;
    world_config.max_cylinders = 8;
    world_config.max_meshes = 0;
    planner_core_ = std::make_unique<curobo_ros_cpp::PlannerCore>(world_config);
    std::string world_error;
    if (!planner_core_->initialize(world_error)) {
      throw std::runtime_error("PlannerCore init failed: " + world_error);
    }
    planner_ = &planner_core_->motion_planner();
    scene_ = &planner_core_->scene();
    curobo::robot::MpcConfig mpc_config;
    mpc_config.trajectory_dt = static_cast<float>(trajectory_dt);
    mpc_ = std::make_unique<curobo::robot::MpcController>(*planner_, mpc_config);
    ik_ = planner_->create_inverse_kinematics(
        curobo::robot::InverseKinematicsConfig{});

    const auto robot = planner_->robot_info();
    dof_ = robot.num_dof;
    tool_count_ = robot.num_tool_frames;
    if (dof_ <= 0) throw std::runtime_error("artifact has no active joints");
    std::vector<std::string> artifact_names;
    for (int i = 0; i < dof_; ++i) artifact_names.push_back(kinematics_->joint_name(i));
    controller_names_ = controller_joints.empty() ? artifact_names : controller_joints;
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
    if (steps_ <= 0 || knots_ <= 0) throw std::runtime_error("MPC layout is empty");
    samples_per_knot_ = std::max(1, (steps_ - 1) / (knots_ + 4));
    command_knots_ = std::max(1, std::min(
        knots_, command_steps_ / samples_per_knot_));
    knot_dt_ = trajectory_dt * samples_per_knot_;
    knots_per_interval_ = std::max(1, static_cast<int>(
        std::llround(command_interval_ / std::max(knot_dt_, 1e-6))));
    knots_per_interval_ = std::min(knots_per_interval_, knots_);

    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_q_), dof_ * sizeof(float)), "q");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_v_), dof_ * sizeof(float)), "v");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_a_), dof_ * sizeof(float)), "a");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_j_), dof_ * sizeof(float)), "j");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_goal_pos_), tool_count_ * 3 * sizeof(float)),
               "goal pos");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_goal_quat_), tool_count_ * 4 * sizeof(float)),
               "goal quat");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_seed_), knots_ * dof_ * sizeof(float)),
               "seed");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_ik_solution_), dof_ * sizeof(float)),
               "ik solution");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_ik_success_), sizeof(uint8_t)),
               "ik success");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_ik_current_), dof_ * sizeof(float)),
               "ik current");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_ik_position_error_), sizeof(float)),
               "ik position error");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_ik_orientation_error_), sizeof(float)),
               "ik orientation error");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_ik_collision_cost_), sizeof(float)),
               "ik collision cost");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_knots_),
                          static_cast<std::size_t>(knots_) * dof_ * sizeof(float)),
               "plan knots");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_q_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "plan q");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_v_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "plan v");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_a_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "plan a");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_j_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "plan j");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_success_), sizeof(uint8_t)),
               "plan success");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_used_rrt_), sizeof(uint8_t)),
               "plan rrt");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_rrt_iters_), sizeof(int32_t)),
               "plan rrt iters");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_cost_), sizeof(float)), "plan cost");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_collision_), sizeof(float)),
               "plan collision");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_dt_), sizeof(float)), "plan dt");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_position_error_), sizeof(float)),
               "plan pose error");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_plan_orientation_error_), sizeof(float)),
               "plan orientation error");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_fk_pos_),
                          static_cast<std::size_t>(tool_count_) * 3 * sizeof(float)),
               "fk pos");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_fk_quat_),
                          static_cast<std::size_t>(tool_count_) * 4 * sizeof(float)),
               "fk quat");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_fk_jac_),
                          static_cast<std::size_t>(tool_count_) * 6 * dof_ * sizeof(float)),
               "fk jac");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_brake_q_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "brake q");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_brake_v_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "brake v");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_brake_a_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "brake a");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_brake_j_),
                          static_cast<std::size_t>(steps_) * dof_ * sizeof(float)),
               "brake j");

    if (adapter_name_ == "jtc") {
      adapter_ = std::make_unique<curobo_execution_cpp::JtcAdapter>(
          this, action_, controller_names_, goal_tolerance_, goal_time_tolerance_);
    } else if (adapter_name_ == "topic") {
      adapter_ = std::make_unique<curobo_execution_cpp::TopicAdapter>(
          this, command_topic_);
    } else {
      throw std::runtime_error("unknown adapter '" + adapter_name_ + "'");
    }
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/status", rclcpp::QoS(1).transient_local());
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic, 10,
        std::bind(&MpcNode::joint_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, std::bind(&MpcNode::goal_callback, this, std::placeholders::_1));
    live_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        live_goal_topic_, 10,
        std::bind(&MpcNode::live_goal_callback, this, std::placeholders::_1));
    live_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        live_goal_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc_goal_marker", 10);
    predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/mpc_predicted_path", 10);
    costs_pub_ = this->create_publisher<curobo_msgs::msg::MpcCosts>(
        "/mpc_costs", 10);
    mpc_goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/unified_planner/mpc_goal", 10,
        std::bind(&MpcNode::mpc_goal_callback, this, std::placeholders::_1));
    set_planner_srv_ = this->create_service<curobo_msgs::srv::SetPlanner>(
        "/unified_planner/set_planner",
        std::bind(&MpcNode::set_planner_callback, this, std::placeholders::_1,
                  std::placeholders::_2));
    get_planners_srv_ = this->create_service<curobo_msgs::srv::GetPlanners>(
        "/unified_planner/get_planners",
        std::bind(&MpcNode::get_planners_callback, this, std::placeholders::_1,
                  std::placeholders::_2));
    generate_srv_ = this->create_service<curobo_msgs::srv::TrajectoryGeneration>(
        "/unified_planner/generate_trajectory",
        std::bind(&MpcNode::generate_trajectory_callback, this, std::placeholders::_1,
                  std::placeholders::_2));
    action_server_ = rclcpp_action::create_server<SendTrajectory>(
        this, "/unified_planner/execute_trajectory",
        std::bind(&MpcNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MpcNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&MpcNode::handle_accepted, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/cancel", std::bind(&MpcNode::cancel_callback, this, std::placeholders::_1,
                              std::placeholders::_2));
    fault_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        fault_topic, 10, std::bind(&MpcNode::fault_callback, this, std::placeholders::_1));
    rclcpp::QoS world_qos(1);
    world_qos.transient_local();
    world_sub_ = this->create_subscription<curobo_planner_interfaces::msg::ObstacleUpdate>(
        world_topic_, world_qos,
        std::bind(&MpcNode::world_update_callback, this, std::placeholders::_1));
    // Event-driven: poll the adapter at 50 Hz and send the next segment as soon
    // as the previous JTC segment is complete. There is no fixed command wait.
    accept_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MpcNode::accept_callback, this));
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MpcNode::diagnostics_callback, this));

    set_status("idle");
    producer_ = std::thread(&MpcNode::producer_loop, this);
    far_replan_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(std::max(far_replan_period_, 0.1)),
        std::bind(&MpcNode::far_replan_callback, this));
    RCLCPP_INFO(this->get_logger(),
                "MPC producer/consumer: dof=%d knots=%d steps=%d control=%.1f Hz "
                "publish=%.1f Hz interval=%.3f s knots_per_interval=%d adapter=%s",
                dof_, knots_, steps_, control_rate_, publish_rate_, command_interval_,
                knots_per_interval_, adapter_name_.c_str());
  }

  ~MpcNode() override {
    running_ = false;
    if (producer_.joinable()) producer_.join();
    cudaFree(d_q_);
    cudaFree(d_v_);
    cudaFree(d_a_);
    cudaFree(d_j_);
    cudaFree(d_goal_pos_);
    cudaFree(d_goal_quat_);
    cudaFree(d_seed_);
    cudaFree(d_ik_solution_);
    cudaFree(d_ik_success_);
    cudaFree(d_ik_current_);
    cudaFree(d_ik_position_error_);
    cudaFree(d_ik_orientation_error_);
    cudaFree(d_ik_collision_cost_);
    cudaFree(d_plan_knots_);
    cudaFree(d_plan_q_);
    cudaFree(d_plan_v_);
    cudaFree(d_plan_a_);
    cudaFree(d_plan_j_);
    cudaFree(d_plan_success_);
    cudaFree(d_plan_used_rrt_);
    cudaFree(d_plan_rrt_iters_);
    cudaFree(d_plan_cost_);
    cudaFree(d_plan_collision_);
    cudaFree(d_plan_dt_);
    cudaFree(d_plan_position_error_);
    cudaFree(d_plan_orientation_error_);
    cudaFree(d_fk_pos_);
    cudaFree(d_fk_quat_);
    cudaFree(d_fk_jac_);
    cudaFree(d_brake_q_);
    cudaFree(d_brake_v_);
    cudaFree(d_brake_a_);
    cudaFree(d_brake_j_);
  }

 private:
  void parse_name_map(const std::string& text) {
    std::size_t start = 0;
    while (start < text.size()) {
      const std::size_t comma = text.find(',', start);
      const std::string entry = text.substr(
          start, comma == std::string::npos ? std::string::npos : comma - start);
      const std::size_t sep = entry.find(':');
      if (sep != std::string::npos) name_map_[entry.substr(0, sep)] = entry.substr(sep + 1);
      if (comma == std::string::npos) break;
      start = comma + 1;
    }
  }

  void set_status(const std::string& status) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_ == status) return;
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto now = std::chrono::steady_clock::now();
    velocity_.clear();
    if (has_state_ && last_state_time_ != std::chrono::steady_clock::time_point{}) {
      const double dt = std::chrono::duration<double>(now - last_state_time_).count();
      if (dt > 1e-5) {
        for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
          const auto mapped = name_map_.find(message->name[i]);
          const std::string name = mapped == name_map_.end() ? message->name[i] : mapped->second;
          const auto old = state_.find(name);
          if (old != state_.end()) {
            velocity_[name] = static_cast<float>(
                (message->position[i] - old->second) / dt);
          }
        }
      }
    }
    state_.clear();
    for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
      const auto mapped = name_map_.find(message->name[i]);
      state_[mapped == name_map_.end() ? message->name[i] : mapped->second] =
          static_cast<float>(message->position[i]);
    }
    last_state_time_ = now;
    has_state_ = true;
  }

  void fault_callback(const std_msgs::msg::Bool::SharedPtr message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    fault_ = message->data;
    if (fault_) set_status("fault");
  }

  void world_update_callback(
      const curobo_planner_interfaces::msg::ObstacleUpdate::SharedPtr message) {
    curobo_ros_cpp::ObjectSpec object;
    object.name = message->name;
    object.position = {static_cast<float>(message->pose.position.x),
                       static_cast<float>(message->pose.position.y),
                       static_cast<float>(message->pose.position.z)};
    object.quaternion = {static_cast<float>(message->pose.orientation.w),
                         static_cast<float>(message->pose.orientation.x),
                         static_cast<float>(message->pose.orientation.y),
                         static_cast<float>(message->pose.orientation.z)};
    object.dimensions = {static_cast<float>(message->dimensions.x),
                         static_cast<float>(message->dimensions.y),
                         static_cast<float>(message->dimensions.z)};
    switch (message->type) {
      case 0: object.type = curobo_ros_cpp::ObjectType::CUBOID; break;
      case 1: object.type = curobo_ros_cpp::ObjectType::SPHERE; break;
      case 2: object.type = curobo_ros_cpp::ObjectType::CAPSULE; break;
      case 3: object.type = curobo_ros_cpp::ObjectType::CYLINDER; break;
      default: return;
    }
    std::lock_guard<std::mutex> lock(mpc_mutex_);
    std::string error;
    if (message->command == curobo_planner_interfaces::msg::ObstacleUpdate::ADD) {
      planner_core_->remove_object(object.name, error);
      if (!planner_core_->add_object(object, error)) {
        RCLCPP_WARN(this->get_logger(), "world add '%s' failed: %s",
                    object.name.c_str(), error.c_str());
        return;
      }
    } else if (message->command == curobo_planner_interfaces::msg::ObstacleUpdate::UPDATE) {
      if (!planner_core_->update_object_pose(object.name, object.position,
                                             object.quaternion, error)) {
        // A late subscriber may have missed the original ADD. Recreate from
        // the UPDATE payload instead of dropping the obstacle.
        if (!planner_core_->add_object(object, error)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "world update '%s' failed: %s",
                               object.name.c_str(), error.c_str());
          return;
        }
      }
    } else if (message->command == curobo_planner_interfaces::msg::ObstacleUpdate::REMOVE) {
      planner_core_->remove_object(object.name, error);
    } else if (message->command == curobo_planner_interfaces::msg::ObstacleUpdate::CLEAR) {
      planner_core_->clear_objects(error);
    } else {
      return;
    }
    if (object.name == "bat") {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      bat_pose_.position = object.position;
      bat_pose_.quaternion = object.quaternion;
      has_bat_pose_ = true;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "world update applied: '%s'", object.name.c_str());
  }

  bool snapshot_state(std::vector<float>& q) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_state_) return false;
    q.resize(static_cast<std::size_t>(dof_));
    for (int i = 0; i < dof_; ++i) {
      const std::string name = kinematics_->joint_name(i);
      const auto it = state_.find(name);
      if (it == state_.end()) return false;
      q[i] = it->second;
    }
    return true;
  }

  void apply_live_goal_locked(const geometry_msgs::msg::Pose& goal,
                              const std::vector<float>& q) {
    const auto& o = goal.orientation;
    const double norm = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    if (!std::isfinite(norm) || norm < 1e-9) return;
    std::vector<float> positions(static_cast<std::size_t>(tool_count_) * 3);
    std::vector<float> quats(static_cast<std::size_t>(tool_count_) * 4);
    for (int tool = 0; tool < tool_count_; ++tool) {
      positions[static_cast<std::size_t>(tool) * 3 + 0] = goal.position.x;
      positions[static_cast<std::size_t>(tool) * 3 + 1] = goal.position.y;
      positions[static_cast<std::size_t>(tool) * 3 + 2] = goal.position.z;
      quats[static_cast<std::size_t>(tool) * 4 + 0] = o.w / norm;
      quats[static_cast<std::size_t>(tool) * 4 + 1] = o.x / norm;
      quats[static_cast<std::size_t>(tool) * 4 + 2] = o.y / norm;
      quats[static_cast<std::size_t>(tool) * 4 + 3] = o.z / norm;
    }

    check_cuda(cudaMemcpy(d_goal_pos_, positions.data(), positions.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "live goal pos");
    check_cuda(cudaMemcpy(d_goal_quat_, quats.data(), quats.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "live goal quat");
    if (live_mode_) {
      mpc_->update_cartesian_target(d_goal_pos_, d_goal_quat_);
      return;
    }

    const auto info = planner_->info();
    std::vector<float> seed(static_cast<std::size_t>(knots_) * dof_);
    curobo::robot::PoseMotionPlanningRequest request;
    check_cuda(cudaMemcpy(d_q_, q.data(), q.size() * sizeof(float), cudaMemcpyHostToDevice),
               "live goal q");
    request.current_position = d_q_;
    request.goal_positions = d_goal_pos_;
    request.goal_quaternions = d_goal_quat_;
    request.optimized_knots = d_plan_knots_;
    request.trajectory_position = d_plan_q_;
    request.trajectory_velocity = d_plan_v_;
    request.trajectory_acceleration = d_plan_a_;
    request.trajectory_jerk = d_plan_j_;
    request.success = d_plan_success_;
    request.used_rrt = d_plan_used_rrt_;
    request.rrt_iterations = d_plan_rrt_iters_;
    request.cost = d_plan_cost_;
    request.collision_cost = d_plan_collision_;
    request.trajectory_dt = d_plan_dt_;
    request.position_error = d_plan_position_error_;
    request.orientation_error = d_plan_orientation_error_;
    planner_->plan_pose(request);
    uint8_t ok = 0;
    check_cuda(cudaMemcpy(&ok, d_plan_success_, 1, cudaMemcpyDeviceToHost),
               "live plan success");
    if (ok != 0) {
      check_cuda(cudaMemcpy(seed.data(), d_plan_knots_,
                            seed.size() * sizeof(float), cudaMemcpyDeviceToHost),
                 "live plan seed");
    } else {
      for (int i = 0; i < knots_; ++i) {
        std::copy(q.begin(), q.end(), seed.begin() + i * dof_);
      }
    }
    check_cuda(cudaMemcpy(d_seed_, seed.data(), seed.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "live seed");
    curobo::robot::MpcCartesianConfig objective;
    objective.constrain_path = false;
    mpc_->reset_cartesian(d_goal_pos_, d_goal_quat_, d_seed_, objective);
    last_action_v_.assign(static_cast<std::size_t>(dof_), 0.0F);
    last_action_a_.assign(static_cast<std::size_t>(dof_), 0.0F);
    live_mode_ = true;
    planner_mode_ = false;
    active_goal_ = true;
    proposal_fresh_ = false;
    executed_knots_ = 1;
    has_predicted_ = false;
    set_status("live tracking");
    (void)info;
  }

  void live_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    const auto& o = message->pose.orientation;
    const double norm = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    if (!std::isfinite(norm) || norm < 1e-9) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_live_pose_ = message->pose;
    has_pending_live_ = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    std::vector<float> q;
    if (!snapshot_state(q)) {
      set_status("waiting for /joint_states");
      return;
    }
    const auto& o = message->pose.orientation;
    const double norm = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    if (!std::isfinite(norm) || norm < 1e-9) {
      set_status("invalid goal quaternion");
      return;
    }
    std::vector<float> positions(static_cast<std::size_t>(tool_count_) * 3);
    std::vector<float> quats(static_cast<std::size_t>(tool_count_) * 4);
    for (int tool = 0; tool < tool_count_; ++tool) {
      positions[static_cast<std::size_t>(tool) * 3 + 0] = message->pose.position.x;
      positions[static_cast<std::size_t>(tool) * 3 + 1] = message->pose.position.y;
      positions[static_cast<std::size_t>(tool) * 3 + 2] = message->pose.position.z;
      quats[static_cast<std::size_t>(tool) * 4 + 0] = o.w / norm;
      quats[static_cast<std::size_t>(tool) * 4 + 1] = o.x / norm;
      quats[static_cast<std::size_t>(tool) * 4 + 2] = o.y / norm;
      quats[static_cast<std::size_t>(tool) * 4 + 3] = o.z / norm;
    }
    std::lock_guard<std::mutex> mpc_lock(mpc_mutex_);
    planner_mode_ = false;
    live_mode_ = false;
    last_action_v_.assign(static_cast<std::size_t>(dof_), 0.0F);
    last_action_a_.assign(static_cast<std::size_t>(dof_), 0.0F);
    check_cuda(cudaMemcpy(d_goal_pos_, positions.data(), positions.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "goal pos");
    check_cuda(cudaMemcpy(d_goal_quat_, quats.data(), quats.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "goal quat");
    check_cuda(cudaMemcpy(d_ik_current_, q.data(), q.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "ik current");

    std::vector<float> goal_q = q;
    bool have_plan_seed = false;
    std::vector<float> seed(static_cast<std::size_t>(knots_) * dof_, 0.0F);
    curobo::robot::PoseMotionPlanningRequest plan_request;
    plan_request.current_position = d_q_;
    plan_request.goal_positions = d_goal_pos_;
    plan_request.goal_quaternions = d_goal_quat_;
    plan_request.optimized_knots = d_plan_knots_;
    plan_request.trajectory_position = d_plan_q_;
    plan_request.trajectory_velocity = d_plan_v_;
    plan_request.trajectory_acceleration = d_plan_a_;
    plan_request.trajectory_jerk = d_plan_j_;
    plan_request.success = d_plan_success_;
    plan_request.used_rrt = d_plan_used_rrt_;
    plan_request.rrt_iterations = d_plan_rrt_iters_;
    plan_request.cost = d_plan_cost_;
    plan_request.collision_cost = d_plan_collision_;
    plan_request.trajectory_dt = d_plan_dt_;
    plan_request.position_error = d_plan_position_error_;
    plan_request.orientation_error = d_plan_orientation_error_;
    check_cuda(cudaMemcpy(d_q_, q.data(), q.size() * sizeof(float), cudaMemcpyHostToDevice),
               "plan start");
    planner_->plan_pose(plan_request);
    uint8_t plan_ok = 0;
    check_cuda(cudaMemcpy(&plan_ok, d_plan_success_, sizeof(uint8_t), cudaMemcpyDeviceToHost),
               "plan success");
    if (plan_ok != 0) {
      check_cuda(cudaMemcpy(seed.data(), d_plan_knots_,
                            seed.size() * sizeof(float), cudaMemcpyDeviceToHost),
                 "plan seed");
      have_plan_seed = true;
      RCLCPP_INFO(this->get_logger(), "MotionPlanner seed found");

      check_cuda(cudaMemcpy(d_q_, q.data(), q.size() * sizeof(float),
                            cudaMemcpyHostToDevice), "fk q");
      kinematics_->forward(d_q_, d_fk_pos_, d_fk_quat_, d_fk_jac_, 1, 1);
      std::vector<float> fk_pos(static_cast<std::size_t>(tool_count_) * 3);
      check_cuda(cudaMemcpy(fk_pos.data(), d_fk_pos_, fk_pos.size() * sizeof(float),
                            cudaMemcpyDeviceToHost), "fk pos");
      const float dx = fk_pos[0] - positions[0];
      const float dy = fk_pos[1] - positions[1];
      const float dz = fk_pos[2] - positions[2];
      const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      RCLCPP_INFO(this->get_logger(),
                  "goal distance=%.3f fk=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f) threshold=%.3f",
                  distance, fk_pos[0], fk_pos[1], fk_pos[2], positions[0], positions[1],
                  positions[2], planner_distance_threshold_);
      if (distance > static_cast<float>(planner_distance_threshold_)) {
        std::vector<float> plan_positions(static_cast<std::size_t>(steps_) * dof_);
        float plan_dt = 0.0F;
        check_cuda(cudaMemcpy(plan_positions.data(), d_plan_q_,
                              plan_positions.size() * sizeof(float), cudaMemcpyDeviceToHost),
                   "plan positions");
        check_cuda(cudaMemcpy(&plan_dt, d_plan_dt_, sizeof(float), cudaMemcpyDeviceToHost),
                   "plan dt");
        trajectory_msgs::msg::JointTrajectory plan_trajectory;
        plan_trajectory.header.stamp = this->now();
        plan_trajectory.joint_names = controller_names_;
        for (int i = 0; i < steps_; ++i) {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions.resize(controller_source_.size());
          for (std::size_t j = 0; j < controller_source_.size(); ++j) {
            point.positions[j] = plan_positions[static_cast<std::size_t>(i) * dof_ +
                                                controller_source_[j]];
          }
          set_duration(point.time_from_start, static_cast<double>(i) * plan_dt);
          plan_trajectory.points.push_back(std::move(point));
        }
        std::string error;
        if (adapter_->send(plan_trajectory, error)) {
          planner_mode_ = true;
          active_goal_ = false;
          proposal_fresh_ = false;
          last_replan_pose_ = bat_pose_;
          has_last_replan_pose_ = has_bat_pose_;
          set_status("planner mode");
          RCLCPP_INFO(this->get_logger(),
                      "far target (%.2f m): executing planner trajectory", distance);
          return;
        }
        RCLCPP_WARN(this->get_logger(), "planner trajectory send failed: %s", error.c_str());
      }
    }

    curobo::robot::IkSolveRequest ik_request;
    ik_request.goal_positions = d_goal_pos_;
    ik_request.goal_quaternions = d_goal_quat_;
    ik_request.current_positions = d_ik_current_;
    ik_request.solution = d_ik_solution_;
    ik_request.success = d_ik_success_;
    ik_request.position_error = d_ik_position_error_;
    ik_request.orientation_error = d_ik_orientation_error_;
    ik_request.collision_cost = d_ik_collision_cost_;
    ik_request.batch_size = 1;
    ik_request.num_goalset = 1;
    ik_request.num_solutions = 1;
    ik_->solve(ik_request);
    uint8_t ik_ok = 0;
    check_cuda(cudaMemcpy(&ik_ok, d_ik_success_, sizeof(uint8_t),
                          cudaMemcpyDeviceToHost), "ik success");
    if (ik_ok != 0) {
      check_cuda(cudaMemcpy(goal_q.data(), d_ik_solution_, goal_q.size() * sizeof(float),
                            cudaMemcpyDeviceToHost), "ik solution");
      RCLCPP_INFO(this->get_logger(), "IK goal found for MPC seed");
    } else {
      RCLCPP_WARN(this->get_logger(), "IK failed; falling back to current-state seed");
    }

    if (!have_plan_seed) {
      for (int k = 0; k < knots_; ++k) {
        const float alpha = knots_ > 1 ? static_cast<float>(k) / (knots_ - 1) : 0.0F;
        for (int j = 0; j < dof_; ++j) {
          seed[static_cast<std::size_t>(k) * dof_ + j] =
              q[j] + alpha * (goal_q[j] - q[j]);
        }
      }
    }
    check_cuda(cudaMemcpy(d_seed_, seed.data(), seed.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "seed");
    if (ik_ok != 0) {
      // Official C++ mpc_test main loop uses a joint goal reset.
      mpc_->reset(d_ik_solution_, d_seed_);
    } else {
      curobo::robot::MpcCartesianConfig objective;
      objective.constrain_path = false;
      mpc_->reset_cartesian(d_goal_pos_, d_goal_quat_, d_seed_, objective);
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    active_goal_ = true;
    proposal_fresh_ = false;
    has_active_ = false;
    has_accept_time_ = false;
    executed_knots_ = 1;
    handoffs_ = 0;
    predicted_q_ = q;
    predicted_v_.assign(static_cast<std::size_t>(dof_), 0.0F);
    predicted_a_.assign(static_cast<std::size_t>(dof_), 0.0F);
    has_predicted_ = true;
    set_status("goal received");
  }

  bool plan_far_locked(const std::vector<float>& q, const char* source) {
    check_cuda(cudaMemcpy(d_q_, q.data(), q.size() * sizeof(float),
                          cudaMemcpyHostToDevice), "far replan current");
    curobo::robot::PoseMotionPlanningRequest request;
    request.current_position = d_q_;
    request.goal_positions = d_goal_pos_;
    request.goal_quaternions = d_goal_quat_;
    request.optimized_knots = d_plan_knots_;
    request.trajectory_position = d_plan_q_;
    request.trajectory_velocity = d_plan_v_;
    request.trajectory_acceleration = d_plan_a_;
    request.trajectory_jerk = d_plan_j_;
    request.success = d_plan_success_;
    request.used_rrt = d_plan_used_rrt_;
    request.rrt_iterations = d_plan_rrt_iters_;
    request.cost = d_plan_cost_;
    request.collision_cost = d_plan_collision_;
    request.trajectory_dt = d_plan_dt_;
    request.position_error = d_plan_position_error_;
    request.orientation_error = d_plan_orientation_error_;
    planner_->plan_pose(request);
    uint8_t ok = 0;
    check_cuda(cudaMemcpy(&ok, d_plan_success_, sizeof(uint8_t), cudaMemcpyDeviceToHost),
               "far replan success");
    if (ok == 0) {
      RCLCPP_WARN(this->get_logger(), "far replan %s failed", source);
      return false;
    }
    std::vector<float> plan_positions(static_cast<std::size_t>(steps_) * dof_);
    float plan_dt = 0.0F;
    check_cuda(cudaMemcpy(plan_positions.data(), d_plan_q_,
                          plan_positions.size() * sizeof(float), cudaMemcpyDeviceToHost),
               "far replan positions");
    check_cuda(cudaMemcpy(&plan_dt, d_plan_dt_, sizeof(float), cudaMemcpyDeviceToHost),
               "far replan dt");
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->now();
    trajectory.joint_names = controller_names_;
    for (int i = 0; i < steps_; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(controller_source_.size());
      for (std::size_t j = 0; j < controller_source_.size(); ++j) {
        point.positions[j] = plan_positions[static_cast<std::size_t>(i) * dof_ +
                                            controller_source_[j]];
      }
      set_duration(point.time_from_start, static_cast<double>(i) * plan_dt);
      trajectory.points.push_back(std::move(point));
    }
    std::string error;
    if (!adapter_->send(trajectory, error)) {
      RCLCPP_WARN(this->get_logger(), "far replan send failed: %s", error.c_str());
      return false;
    }
    planner_mode_ = true;
    active_goal_ = false;
    proposal_fresh_ = false;
    set_status("planner replan");
    RCLCPP_INFO(this->get_logger(), "far replan %s sent", source);
    return true;
  }

  void far_replan_callback() {
    std::vector<float> q;
    if (!snapshot_state(q)) return;
    curobo::robot::Pose bat_pose;
    bool has_bat = false;
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      bat_pose = bat_pose_;
      has_bat = has_bat_pose_;
    }
    std::lock_guard<std::mutex> lock(mpc_mutex_);
    if (!planner_mode_) return;
    if (has_bat && has_last_replan_pose_) {
      const float dx = bat_pose.position[0] - last_replan_pose_.position[0];
      const float dy = bat_pose.position[1] - last_replan_pose_.position[1];
      const float dz = bat_pose.position[2] - last_replan_pose_.position[2];
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <
          static_cast<float>(far_replan_min_move_)) {
        return;
      }
    }
    if (plan_far_locked(q, "dynamic scene")) {
      last_replan_pose_ = bat_pose;
      has_last_replan_pose_ = true;
    }
  }

  void producer_loop() {
    const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_, 1.0));
    auto next = std::chrono::steady_clock::now();
    while (running_) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      geometry_msgs::msg::Pose pending_live;
      bool has_pending_live = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_pending_live_) {
          pending_live = pending_live_pose_;
          has_pending_live_ = false;
          has_pending_live = true;
        }
      }
      if (has_pending_live) {
        std::vector<float> live_q;
        if (snapshot_state(live_q)) {
          std::lock_guard<std::mutex> mpc_lock(mpc_mutex_);
          apply_live_goal_locked(pending_live, live_q);
        }
      }
      std::vector<float> q, v, a, goal_pos, goal_quat, seed;
      int executed = 0;
      bool active = false;
      curobo::robot::Pose bat_pose;
      bool has_bat = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        active = active_goal_;
        executed = executed_knots_;
        bat_pose = bat_pose_;
        has_bat = has_bat_pose_;
        if (active && use_measured_state_ && has_state_) {
          q.resize(static_cast<std::size_t>(dof_));
          v.assign(static_cast<std::size_t>(dof_), 0.0F);
          a.assign(static_cast<std::size_t>(dof_), 0.0F);
          for (int i = 0; i < dof_; ++i) {
            const std::string name = kinematics_->joint_name(i);
            const auto q_it = state_.find(name);
            if (q_it == state_.end()) { active = false; break; }
            q[i] = q_it->second;
            const auto v_it = velocity_.find(name);
            v[i] = v_it != velocity_.end() ? v_it->second : last_action_v_[i];
            a[i] = last_action_a_[i];
          }
        } else if (active && has_predicted_) {
          q = predicted_q_;
          v = predicted_v_;
          a = predicted_a_;
        } else if (active && has_state_) {
          q.resize(static_cast<std::size_t>(dof_));
          v.assign(static_cast<std::size_t>(dof_), 0.0F);
          a.assign(static_cast<std::size_t>(dof_), 0.0F);
          for (int i = 0; i < dof_; ++i) {
            const auto it = state_.find(kinematics_->joint_name(i));
            if (it == state_.end()) { active = false; break; }
            q[i] = it->second;
          }
        } else {
          active = false;
        }
      }
      if (active && !adapter_->active()) {
        try {
          std::lock_guard<std::mutex> mpc_lock(mpc_mutex_);
          std::vector<float> zero(static_cast<std::size_t>(dof_), 0.0F);
          check_cuda(cudaMemcpy(d_q_, q.data(), q.size() * sizeof(float),
                                cudaMemcpyHostToDevice), "q");
          check_cuda(cudaMemcpy(d_v_, v.data(), v.size() * sizeof(float),
                                cudaMemcpyHostToDevice), "v");
          check_cuda(cudaMemcpy(d_a_, a.data(), a.size() * sizeof(float),
                                cudaMemcpyHostToDevice), "a");
          check_cuda(cudaMemset(d_j_, 0, zero.size() * sizeof(float)), "j");
          const auto result = mpc_->step(d_q_, d_v_, d_a_, d_j_, executed);
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_cost_ = result.cost;
            last_collision_ = result.collision_cost;
          }
          if (!result.feasible) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "MPC infeasible: cost=%.3f collision=%.3f knots=%d",
                                 result.cost, result.collision_cost, executed);
            std::lock_guard<std::mutex> lock(data_mutex_);
            executed_knots_ = 0;
            proposal_fresh_ = false;
            proposal_.clear();
            proposal_v_.clear();
            proposal_a_.clear();
          }
          if (result.feasible) {
            // NOTE: do not validate the raw optimizer tensors here. The
            // official C++ test builds a quintic execution spline first and
            // validates that spline; raw candidate validation rejects valid
            // proposals. Keep the candidate for the execution adapter.
            std::vector<float> horizon(static_cast<std::size_t>(steps_) * dof_);
            std::vector<float> horizon_v(horizon.size());
            std::vector<float> horizon_a(horizon.size());
            check_cuda(cudaMemcpy(horizon.data(), result.position,
                                  horizon.size() * sizeof(float), cudaMemcpyDeviceToHost),
                       "horizon");
            check_cuda(cudaMemcpy(horizon_v.data(), result.velocity,
                                  horizon_v.size() * sizeof(float), cudaMemcpyDeviceToHost),
                       "horizon v");
            check_cuda(cudaMemcpy(horizon_a.data(), result.acceleration,
                                  horizon_a.size() * sizeof(float), cudaMemcpyDeviceToHost),
                       "horizon a");
            std::lock_guard<std::mutex> lock(data_mutex_);
            proposal_ = std::move(horizon);
            proposal_v_ = std::move(horizon_v);
            proposal_a_ = std::move(horizon_a);
            proposal_dt_ = result.trajectory_dt;
            proposal_fresh_ = true;
            executed_knots_ = 0;  // shift only once per accepted segment
          }
        } catch (const std::exception& exception) {
          set_status(std::string("producer error: ") + exception.what());
        }
      }
      std::this_thread::sleep_until(next);
    }
  }

  bool admission_ok(std::string& reason) {
    std::vector<float> q(static_cast<std::size_t>(dof_), 0.0F);
    std::vector<float> velocity(static_cast<std::size_t>(dof_), 0.0F);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!active_goal_ || !proposal_fresh_ || proposal_.empty()) {
        reason = "no fresh proposal";
        return false;
      }
      if (!has_state_ ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_state_time_)
                  .count() > state_timeout_) {
        reason = "stale joint state";
        return false;
      }
      if (fault_) {
        reason = "controller fault";
        return false;
      }
      for (int i = 0; i < dof_; ++i) {
        const std::string name = kinematics_->joint_name(i);
        const auto state_it = state_.find(name);
        if (state_it == state_.end()) {
          reason = "missing joint " + name;
          return false;
        }
        q[i] = state_it->second;
        const auto vel_it = velocity_.find(name);
        if (vel_it != velocity_.end()) velocity[i] = vel_it->second;
      }
      // Official C++ MPC validates the candidate trajectory; it does not gate
      // acceptance on measured-vs-predicted tracking error. Keep measured
      // state only for freshness/fault/brake checks.
    }
    if (!brake_feasible(q, velocity)) {
      reason = "brake infeasible";
      return false;
    }
    return true;
  }

  bool brake_feasible(const std::vector<float>& q, const std::vector<float>& velocity) {
    if (!brake_check_ || brake_deceleration_ <= 0.0) return true;
    float max_speed = 0.0F;
    for (float value : velocity) max_speed = std::max(max_speed, std::abs(value));
    if (max_speed < 1e-3F) return true;

    const float dt = active_dt_ > 0.0F ? active_dt_ : 0.03F;
    const std::size_t count = static_cast<std::size_t>(steps_) * dof_;
    std::vector<float> bq(count), bv(count), ba(count), bj(count);
    std::vector<float> previous_a(static_cast<std::size_t>(dof_), 0.0F);
    for (int i = 0; i < steps_; ++i) {
      const float t = static_cast<float>(i) * dt;
      for (int j = 0; j < dof_; ++j) {
        const float v0 = velocity[j];
        const float a = -std::copysign(static_cast<float>(brake_deceleration_), v0);
        const float stop_time = std::abs(v0) / static_cast<float>(brake_deceleration_);
        const float tj = std::min(t, stop_time);
        const std::size_t index = static_cast<std::size_t>(i) * dof_ + j;
        bq[index] = q[j] + v0 * tj + 0.5F * a * tj * tj;
        bv[index] = t >= stop_time ? 0.0F : v0 + a * t;
        ba[index] = t >= stop_time ? 0.0F : a;
        bj[index] = (ba[index] - previous_a[j]) / dt;
        previous_a[j] = ba[index];
      }
    }
    check_cuda(cudaMemcpy(d_brake_q_, bq.data(), count * sizeof(float),
                          cudaMemcpyHostToDevice), "brake q");
    check_cuda(cudaMemcpy(d_brake_v_, bv.data(), count * sizeof(float),
                          cudaMemcpyHostToDevice), "brake v");
    check_cuda(cudaMemcpy(d_brake_a_, ba.data(), count * sizeof(float),
                          cudaMemcpyHostToDevice), "brake a");
    check_cuda(cudaMemcpy(d_brake_j_, bj.data(), count * sizeof(float),
                          cudaMemcpyHostToDevice), "brake j");
    return mpc_->validate_trajectory(d_brake_q_, d_brake_v_, d_brake_a_, d_brake_j_, dt);
  }

  void accept_callback() {
    std::lock_guard<std::mutex> mpc_lock(mpc_mutex_);
    if (adapter_->active()) {
      return;  // previous segment still executing; no fixed wait
    }
    std::string reason;
    if (!admission_ok(reason)) {
      set_status("holding: " + reason);
      return;
    }

    trajectory_msgs::msg::JointTrajectory segment;
    segment.header.stamp = this->now();
    segment.joint_names = controller_names_;
    const int segment_steps = std::max(
        2, std::min(steps_, command_steps_ + 1));
    for (int i = 0; i < segment_steps; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(controller_source_.size());
      for (std::size_t j = 0; j < controller_source_.size(); ++j) {
        point.positions[j] = proposal_[static_cast<std::size_t>(i) * dof_ +
                                       controller_source_[j]];
      }
      set_duration(point.time_from_start, static_cast<double>(i) * proposal_dt_);
      segment.points.push_back(std::move(point));
    }

    if (!segment.points.empty()) {
      float measured = 0.0F;
      {
        std::lock_guard<std::mutex> data_lock(data_mutex_);
        const auto it = state_.find(kinematics_->joint_name(0));
        if (it != state_.end()) measured = it->second;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "MPC segment q0 first=%.4f last=%.4f measured=%.4f "
                           "executed=%d use_measured=%s",
                           segment.points.front().positions[0],
                           segment.points.back().positions[0], measured,
                           executed_knots_, use_measured_state_ ? "true" : "false");
    }
    std::string error;
    if (!adapter_->send(segment, error)) {
      set_status("send failed: " + error);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      const int period_index = std::min(steps_ - 1, command_steps_);
      predicted_q_.resize(static_cast<std::size_t>(dof_));
      predicted_v_.resize(static_cast<std::size_t>(dof_));
      predicted_a_.resize(static_cast<std::size_t>(dof_));
      for (int j = 0; j < dof_; ++j) {
        predicted_q_[j] = proposal_[static_cast<std::size_t>(period_index) * dof_ + j];
        predicted_v_[j] = proposal_v_[static_cast<std::size_t>(period_index) * dof_ + j];
        predicted_a_[j] = proposal_a_[static_cast<std::size_t>(period_index) * dof_ + j];
      }
      has_predicted_ = true;
      last_action_v_.resize(static_cast<std::size_t>(dof_));
      last_action_a_.resize(static_cast<std::size_t>(dof_));
      for (int j = 0; j < dof_; ++j) {
        last_action_v_[j] = proposal_v_[static_cast<std::size_t>(period_index) * dof_ + j];
        last_action_a_[j] = proposal_a_[static_cast<std::size_t>(period_index) * dof_ + j];
      }
      mpc_->accept();
      active_ = proposal_;
      active_dt_ = proposal_dt_;
      active_start_ = std::chrono::steady_clock::now();
      last_accept_time_ = active_start_;
      has_accept_time_ = true;
      has_active_ = true;
      proposal_fresh_ = false;
      executed_knots_ = command_knots_;
      ++handoffs_;
    }
    set_status("tracking");
  }

  geometry_msgs::msg::PoseStamped make_pose_stamped(
      const geometry_msgs::msg::Pose& pose) const {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = base_frame_;
    message.header.stamp = this->now();
    message.pose = pose;
    return message;
  }

  void set_planner_callback(
      const std::shared_ptr<curobo_msgs::srv::SetPlanner::Request> request,
      std::shared_ptr<curobo_msgs::srv::SetPlanner::Response> response) {
    if (request->planner_type == curobo_msgs::srv::SetPlanner::Request::MPC) {
      response->success = true;
      response->previous_planner = current_planner_;
      current_planner_ = "Model Predictive Control (MPC)";
      response->current_planner = current_planner_;
      response->message = "MPC selected";
    } else {
      response->success = false;
      response->previous_planner = current_planner_;
      response->current_planner = current_planner_;
      response->message = "only MPC is implemented";
    }
  }

  void get_planners_callback(
      const std::shared_ptr<curobo_msgs::srv::GetPlanners::Request>,
      std::shared_ptr<curobo_msgs::srv::GetPlanners::Response> response) {
    response->planner_names = {"Model Predictive Control (MPC)"};
    response->planner_ids = {curobo_msgs::srv::SetPlanner::Request::MPC};
    response->current_planner_name = current_planner_;
    response->current_planner_id = curobo_msgs::srv::SetPlanner::Request::MPC;
    response->success = true;
  }

  void generate_trajectory_callback(
      const std::shared_ptr<curobo_msgs::srv::TrajectoryGeneration::Request> request,
      std::shared_ptr<curobo_msgs::srv::TrajectoryGeneration::Response> response) {
    auto message = std::make_shared<geometry_msgs::msg::PoseStamped>(
        make_pose_stamped(request->target_pose));
    goal_callback(message);
    response->success = true;
    response->message = "MPC target accepted; no open-loop trajectory";
    response->dt = 0.0;
  }

  void publish_goal_marker(const geometry_msgs::msg::Pose& pose) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = this->now();
    marker.ns = "mpc_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
    marker.color.g = 1.0;
    marker.color.a = 0.8;
    marker_pub_->publish(marker);
  }

  void mpc_goal_callback(const geometry_msgs::msg::Pose::SharedPtr message) {
    if (!action_active_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "mpc_goal ignored: no active execute_trajectory action");
      return;
    }
    publish_goal_marker(*message);
    auto stamped = std::make_shared<geometry_msgs::msg::PoseStamped>(
        make_pose_stamped(*message));
    live_goal_callback(stamped);
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const SendTrajectory::Goal> goal) {
    if (action_active_) return rclcpp_action::GoalResponse::REJECT;
    if (goal->target_pose.position.x == 0.0 && goal->target_pose.position.y == 0.0 &&
        goal->target_pose.position.z == 0.0) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    action_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    action_active_ = false;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    cancel_callback(request, response);
    auto result = std::make_shared<SendTrajectory::Result>();
    result->success = true;
    result->message = "canceled";
    goal_handle->canceled(result);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    action_active_ = true;
    feedback_hold_count_ = 0;
    action_target_ = goal_handle->get_goal()->target_pose;
    publish_goal_marker(action_target_);
    live_goal_pub_->publish(make_pose_stamped(action_target_));
    action_feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        [this, goal_handle]() { publish_action_feedback(goal_handle); });
  }

  void publish_action_feedback(const std::shared_ptr<GoalHandle> goal_handle) {
    if (!action_active_) return;
    auto feedback = std::make_shared<SendTrajectory::Feedback>();
    feedback->state = "TRACKING";
    feedback->position_error = 0.0;
    feedback->on_target = false;
    goal_handle->publish_feedback(feedback);
  }

  void diagnostics_callback() {
    if (!action_active_) return;
    float fk_error = 0.0F;
    float rot_error = 0.0F;
    float collision = 0.0F;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      fk_error = last_fk_error_;
      rot_error = last_orientation_error_;
      collision = last_collision_;
    }
    curobo_msgs::msg::MpcCosts costs;
    costs.header.frame_id = base_frame_;
    costs.header.stamp = this->now();
    costs.fk_err_m = fk_error;
    costs.fk_rot_err_deg = rot_error * 57.2957795;
    costs.pose_pos_err_m = fk_error;
    costs.pose_rot_err_rad = rot_error;
    costs.con_scene_collision = collision;
    costs_pub_->publish(costs);
  }

  void cancel_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      active_goal_ = false;
      proposal_fresh_ = false;
      has_active_ = false;
    }
    response->success = true;
    response->message = "MPC canceled";
    set_status("canceled");
  }

  std::string artifact_dir_;
  std::string command_topic_;
  std::string adapter_name_;
  std::string action_;
  std::string world_topic_;
  std::string live_goal_topic_;
  std::string base_frame_;
  std::string tool_frame_;
  std::string current_planner_ = "Model Predictive Control (MPC)";
  std::atomic<bool> action_active_{false};
  int feedback_hold_count_ = 0;
  geometry_msgs::msg::Pose action_target_;
  double bat_length_ = 0.6;
  double bat_radius_ = 0.03;
  double bat_initial_x_ = 0.45;
  double bat_initial_y_ = 0.0;
  double bat_initial_z_ = 0.55;
  double publish_rate_ = 100.0;
  double goal_tolerance_ = 0.02;
  double goal_time_tolerance_ = 2.0;
  double control_rate_ = 30.0;
  double command_interval_ = 0.48;
  double state_timeout_ = 0.2;
  double max_tracking_error_ = 0.05;
  double brake_deceleration_ = 2.0;
  bool brake_check_ = true;
  double knot_dt_ = 0.12;
  int knots_per_interval_ = 2;
  int dof_ = 0;
  int tool_count_ = 0;
  int steps_ = 0;
  int knots_ = 0;
  int command_steps_ = 16;
  int samples_per_knot_ = 4;
  int command_knots_ = 4;
  bool use_measured_state_ = false;

  std::unique_ptr<curobo::robot::Kinematics> kinematics_;
  std::unique_ptr<curobo_ros_cpp::PlannerCore> planner_core_;
  curobo::robot::Scene* scene_ = nullptr;
  curobo::robot::MotionPlanner* planner_ = nullptr;
  std::unique_ptr<curobo::robot::MpcController> mpc_;
  std::unique_ptr<curobo::robot::InverseKinematics> ik_;

  std::unique_ptr<curobo_execution_cpp::ExecutionAdapter> adapter_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr live_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_sub_;
  rclcpp::Subscription<curobo_planner_interfaces::msg::ObstacleUpdate>::SharedPtr world_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<curobo_msgs::srv::SetPlanner>::SharedPtr set_planner_srv_;
  rclcpp::Service<curobo_msgs::srv::GetPlanners>::SharedPtr get_planners_srv_;
  rclcpp::Service<curobo_msgs::srv::TrajectoryGeneration>::SharedPtr generate_srv_;
  rclcpp_action::Server<SendTrajectory>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr action_feedback_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr mpc_goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr live_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<curobo_msgs::msg::MpcCosts>::SharedPtr costs_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr accept_timer_;
  rclcpp::TimerBase::SharedPtr far_replan_timer_;
  std::thread producer_;
  std::atomic<bool> running_{true};

  std::vector<std::string> controller_names_;
  std::vector<std::size_t> controller_source_;
  std::unordered_map<std::string, std::string> name_map_;

  std::mutex mpc_mutex_;
  std::mutex data_mutex_;
  std::mutex status_mutex_;
  std::unordered_map<std::string, float> state_;
  std::unordered_map<std::string, float> velocity_;
  bool has_state_ = false;
  bool fault_ = false;
  bool has_bat_pose_ = false;
  bool planner_mode_ = false;
  bool live_mode_ = false;
  bool has_pending_live_ = false;
  geometry_msgs::msg::Pose pending_live_pose_;
  double planner_distance_threshold_ = 0.15;
  double far_replan_period_ = 1.0;
  double far_replan_min_move_ = 0.01;
  curobo::robot::Pose last_replan_pose_;
  bool has_last_replan_pose_ = false;
  curobo::robot::Pose bat_pose_;
  std::chrono::steady_clock::time_point last_state_time_{};

  bool active_goal_ = false;
  bool proposal_fresh_ = false;
  bool has_active_ = false;
  bool has_accept_time_ = false;
  std::chrono::steady_clock::time_point last_accept_time_{};
  int executed_knots_ = 0;
  int handoffs_ = 0;
  std::vector<float> proposal_;
  std::vector<float> proposal_v_;
  std::vector<float> proposal_a_;
  float last_cost_ = 0.0F;
  float last_collision_ = 0.0F;
  float last_fk_error_ = 0.0F;
  float last_orientation_error_ = 0.0F;
  std::vector<float> last_action_v_;
  std::vector<float> last_action_a_;
  std::vector<float> predicted_q_;
  std::vector<float> predicted_v_;
  std::vector<float> predicted_a_;
  bool has_predicted_ = false;
  float proposal_dt_ = 0.0F;
  std::vector<float> active_;
  float active_dt_ = 0.0F;
  std::chrono::steady_clock::time_point active_start_{};
  std::string status_ = "idle";

  float* d_q_ = nullptr;
  float* d_v_ = nullptr;
  float* d_a_ = nullptr;
  float* d_j_ = nullptr;
  float* d_goal_pos_ = nullptr;
  float* d_goal_quat_ = nullptr;
  float* d_seed_ = nullptr;
  float* d_ik_solution_ = nullptr;
  uint8_t* d_ik_success_ = nullptr;
  float* d_ik_current_ = nullptr;
  float* d_ik_position_error_ = nullptr;
  float* d_ik_orientation_error_ = nullptr;
  float* d_ik_collision_cost_ = nullptr;
  float* d_plan_knots_ = nullptr;
  float* d_plan_q_ = nullptr;
  float* d_plan_v_ = nullptr;
  float* d_plan_a_ = nullptr;
  float* d_plan_j_ = nullptr;
  uint8_t* d_plan_success_ = nullptr;
  uint8_t* d_plan_used_rrt_ = nullptr;
  int32_t* d_plan_rrt_iters_ = nullptr;
  float* d_plan_cost_ = nullptr;
  float* d_plan_collision_ = nullptr;
  float* d_plan_dt_ = nullptr;
  float* d_plan_position_error_ = nullptr;
  float* d_plan_orientation_error_ = nullptr;
  float* d_fk_pos_ = nullptr;
  float* d_fk_quat_ = nullptr;
  float* d_fk_jac_ = nullptr;
  float* d_brake_q_ = nullptr;
  float* d_brake_v_ = nullptr;
  float* d_brake_a_ = nullptr;
  float* d_brake_j_ = nullptr;
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
