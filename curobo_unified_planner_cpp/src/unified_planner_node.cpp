// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <cmath>
#include <std_srvs/srv/trigger.hpp>
#include "tf2/convert.h"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/time.h"

#include "curobo_msgs/action/send_trajectory.hpp"
#include "curobo_msgs/srv/get_planners.hpp"
#include "curobo_msgs/srv/set_planner.hpp"
#include "curobo_msgs/srv/trajectory_generation.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace curobo_ros_cpp {

class UnifiedPlanner : public rclcpp::Node {
 public:
  using SendTrajectory = curobo_msgs::action::SendTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SendTrajectory>;

  UnifiedPlanner() : Node("unified_planner") {
    base_frame_ = declare_parameter<std::string>("base_frame", "panda_link0");
    tool_frame_ = declare_parameter<std::string>("tool_frame", "panda_hand");
    mpc_goal_topic_ = declare_parameter<std::string>("mpc_goal_topic", "/curobo_mpc/live_goal");
    point_goal_topic_ = declare_parameter<std::string>("point_goal_topic", "/curobo_mpc/goal");
    cancel_service_ = declare_parameter<std::string>("cancel_service", "/curobo_mpc/cancel");

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(point_goal_topic_, 10);
    live_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(mpc_goal_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/mpc_goal_marker", 10);
    cancel_client_ = create_client<std_srvs::srv::Trigger>(cancel_service_);

    set_planner_srv_ = create_service<curobo_msgs::srv::SetPlanner>(
        "/unified_planner/set_planner",
        std::bind(&UnifiedPlanner::set_planner, this, std::placeholders::_1,
                  std::placeholders::_2));
    get_planners_srv_ = create_service<curobo_msgs::srv::GetPlanners>(
        "/unified_planner/get_planners",
        std::bind(&UnifiedPlanner::get_planners, this, std::placeholders::_1,
                  std::placeholders::_2));
    generate_srv_ = create_service<curobo_msgs::srv::TrajectoryGeneration>(
        "/unified_planner/generate_trajectory",
        std::bind(&UnifiedPlanner::generate_trajectory, this, std::placeholders::_1,
                  std::placeholders::_2));
    action_server_ = rclcpp_action::create_server<SendTrajectory>(
        this, "/unified_planner/execute_trajectory",
        std::bind(&UnifiedPlanner::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&UnifiedPlanner::handle_cancel, this, std::placeholders::_1),
        std::bind(&UnifiedPlanner::handle_accepted, this, std::placeholders::_1));

    mpc_goal_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/unified_planner/mpc_goal", 10,
        [this](const geometry_msgs::msg::Pose::SharedPtr message) {
          if (!action_active_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "mpc_goal ignored: no active MPC action");
            return;
          }
          publish_goal(*message, true);
        });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(get_logger(), "unified planner facade ready (MPC id=1)");
  }

 private:
  void set_planner(const std::shared_ptr<curobo_msgs::srv::SetPlanner::Request> request,
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
      response->message = "only MPC is implemented in this facade";
    }
  }

  void get_planners(const std::shared_ptr<curobo_msgs::srv::GetPlanners::Request>,
                    std::shared_ptr<curobo_msgs::srv::GetPlanners::Response> response) {
    response->planner_names = {"Model Predictive Control (MPC)"};
    response->planner_ids = {curobo_msgs::srv::SetPlanner::Request::MPC};
    response->current_planner_name = current_planner_;
    response->current_planner_id = curobo_msgs::srv::SetPlanner::Request::MPC;
    response->success = true;
  }

  void publish_goal(const geometry_msgs::msg::Pose& pose, bool live) {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = base_frame_;
    message.header.stamp = now();
    message.pose = pose;
    (live ? live_pub_ : goal_pub_)->publish(message);
    visualization_msgs::msg::Marker marker;
    marker.header = message.header;
    marker.ns = "mpc_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = message.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
    marker.color.g = 1.0;
    marker.color.a = 0.8;
    marker_pub_->publish(marker);
  }

  void generate_trajectory(
      const std::shared_ptr<curobo_msgs::srv::TrajectoryGeneration::Request> request,
      std::shared_ptr<curobo_msgs::srv::TrajectoryGeneration::Response> response) {
    publish_goal(request->target_pose, false);
    response->success = true;
    response->message = "MPC target accepted; no open-loop trajectory in MPC mode";
    response->dt = 0.0;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const SendTrajectory::Goal> goal) {
    if (action_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_pose.position.x == 0.0 && goal->target_pose.position.y == 0.0 &&
        goal->target_pose.position.z == 0.0) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    action_active_ = false;
    if (cancel_client_->service_is_ready()) {
      cancel_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    action_active_ = true;
    target_ = goal_handle->get_goal()->target_pose;
    publish_goal(target_, true);
    feedback_timer_ = create_wall_timer(
        std::chrono::milliseconds(33),
        [this, goal_handle]() { feedback(goal_handle); });
  }

  void feedback(const std::shared_ptr<GoalHandle> goal_handle) {
    if (!action_active_) return;
    auto feedback = std::make_shared<SendTrajectory::Feedback>();
    feedback->state = "TRACKING";
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero);
    } catch (const std::exception&) {
      feedback->state = "FAILED";
      feedback->position_error = -1.0;
      goal_handle->publish_feedback(feedback);
      return;
    }
    const auto& p = transform.transform.translation;
    const double dx = p.x - target_.position.x;
    const double dy = p.y - target_.position.y;
    const double dz = p.z - target_.position.z;
    feedback->position_error_x = dx;
    feedback->position_error_y = dy;
    feedback->position_error_z = dz;
    feedback->position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    feedback->on_target = feedback->position_error < 0.01;
    feedback->hold_count = feedback->on_target ? feedback->hold_count + 1 : 0;
    if (feedback->on_target) feedback->state = "ON_TARGET";
    goal_handle->publish_feedback(feedback);
  }

  std::string base_frame_, tool_frame_, mpc_goal_topic_, point_goal_topic_, cancel_service_;
  std::string current_planner_ = "Model Predictive Control (MPC)";
  std::atomic<bool> action_active_{false};
  geometry_msgs::msg::Pose target_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_, live_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr mpc_goal_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
  rclcpp::Service<curobo_msgs::srv::SetPlanner>::SharedPtr set_planner_srv_;
  rclcpp::Service<curobo_msgs::srv::GetPlanners>::SharedPtr get_planners_srv_;
  rclcpp::Service<curobo_msgs::srv::TrajectoryGeneration>::SharedPtr generate_srv_;
  rclcpp_action::Server<SendTrajectory>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace curobo_ros_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<curobo_ros_cpp::UnifiedPlanner>());
  rclcpp::shutdown();
  return 0;
}
