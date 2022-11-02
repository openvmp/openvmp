/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-31
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "openvmp_robot/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace openvmp_robot {

// class FibonacciActionClient : public rclcpp::Node {
//  public:
//   using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
//   using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

//  private:
//   rclcpp::TimerBase::SharedPtr timer_;

//   void goal_response_callback(
//       std::shared_future<GoalHandleFibonacci::SharedPtr> future) {
//     auto goal_handle = future.get();
//     if (!goal_handle) {
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//       RCLCPP_INFO(this->get_logger(),
//                   "Goal accepted by server, waiting for result");
//     }
//   }

//   void feedback_callback(
//       GoalHandleFibonacci::SharedPtr,
//       const std::shared_ptr<const Fibonacci::Feedback> feedback) {
//     std::stringstream ss;
//     ss << "Next number in sequence received: ";
//     for (auto number : feedback->partial_sequence) {
//       ss << number << " ";
//     }
//     RCLCPP_INFO(this->get_logger(), ss.str().c_str());
//   }

//   void result_callback(const GoalHandleFibonacci::WrappedResult& result) {
//     switch (result.code) {
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         return;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//         return;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//         return;
//     }
//     std::stringstream ss;
//     ss << "Result received: ";
//     for (auto number : result.result->sequence) {
//       ss << number << " ";
//     }
//     RCLCPP_INFO(this->get_logger(), ss.str().c_str());
//     rclcpp::shutdown();
//   }
// };  // class FibonacciActionClient

void goal_response_callback(
    // Node *that, std::shared_future<rclcpp_action::ClientGoalHandle<
    //                 control_msgs::action::FollowJointTrajectory>::SharedPtr>
    //                 future) {
    Node *that,
    rclcpp_action::ClientGoalHandle<
        control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle) {
  // auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(that->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(that->get_logger(), "Goal accepted by server, waiting...");
  }
};

void feedback_callback(
    Node *that,
    rclcpp_action::ClientGoalHandle<
        control_msgs::action::FollowJointTrajectory>::SharedPtr,
    const std::shared_ptr<
        const control_msgs::action::FollowJointTrajectory::Feedback>
        feedback) {
  RCLCPP_INFO(that->get_logger(), "Next feedback received: %s",
              feedback->header.frame_id.c_str());
};

void result_callback(
    Node *that,
    const rclcpp_action::ClientGoalHandle<
        control_msgs::action::FollowJointTrajectory>::WrappedResult &result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(that->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(that->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(that->get_logger(), "Unknown result code");
      return;
  }
  RCLCPP_INFO(that->get_logger(), "Result received");
};

void Node::preset_motion_walk() {
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      client_ptr;

  client_ptr =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
          this, "trajectory_controller/follow_joint_trajectory");
  if (!client_ptr->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names.push_back("front_turn_table_joint");
  goal_msg.trajectory.joint_names.push_back("front_body_joint");
  goal_msg.trajectory.joint_names.push_back("front_left_arm_joint");
  goal_msg.trajectory.joint_names.push_back("front_left_arm_inner_joint");
  goal_msg.trajectory.joint_names.push_back("front_right_arm_joint");
  goal_msg.trajectory.joint_names.push_back("front_right_arm_inner_joint");
  goal_msg.trajectory.joint_names.push_back("rear_turn_table_joint");
  goal_msg.trajectory.joint_names.push_back("rear_body_joint");
  goal_msg.trajectory.joint_names.push_back("rear_left_arm_joint");
  goal_msg.trajectory.joint_names.push_back("rear_left_arm_inner_joint");
  goal_msg.trajectory.joint_names.push_back("rear_right_arm_joint");
  goal_msg.trajectory.joint_names.push_back("rear_right_arm_inner_joint");
  goal_msg.trajectory.points.resize(3);
  goal_msg.trajectory.points[0].positions = {0, 0.5, 0, 0.5, 0, 0.5,
                                             0, 0.5, 0, 0.5, 0, 0.5};
  goal_msg.trajectory.points[0].velocities = {0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0};
  goal_msg.trajectory.points[0].time_from_start.sec = 1;
  goal_msg.trajectory.points[1].positions = {0.5, 0, 0.5, 0, 0.5, 0,
                                             0.5, 0, 0.5, 0, 0.5, 0};
  goal_msg.trajectory.points[1].velocities = {0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0};
  goal_msg.trajectory.points[1].time_from_start.sec = 2;
  goal_msg.trajectory.points[3].positions = {0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0};
  goal_msg.trajectory.points[3].velocities = {0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0};
  goal_msg.trajectory.points[3].time_from_start.sec = 3;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
      &feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&result_callback, this, std::placeholders::_1);

  client_ptr->async_send_goal(goal_msg, send_goal_options);
}

}  // namespace openvmp_robot
