// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <atomic>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "battery_utils.hpp"
#include "omni_plan/pddl/action.hpp"
#include "poirot/poirot.hpp"
#include "pyrobosim_msgs/action/execute_task_action.hpp"
#include "pyrobosim_msgs/msg/execution_result.hpp"
#include "pyrobosim_msgs/msg/task_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace omni_plan;
using ExecuteTaskAction = pyrobosim_msgs::action::ExecuteTaskAction;

/// @brief Pick up a food item from a location in the pyrobosim restaurant.
/// PDDL: (pick ?robot ?item ?location)
class PickAction : public pddl::Action {
public:
  PickAction()
      : Action("pick", 5,
               {
                   {"robot", "robot"},
                   {"item", "item"},
                   {"location", "location"},
               }) {

    // Preconditions
    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "location"});
    this->add_condition(pddl::START, "item_at",
                        std::vector<std::string>{"item", "location"});
    this->add_condition(pddl::START, "gripper_free",
                        std::vector<std::string>{"robot"});
    this->add_condition(pddl::START, "battery_ok",
                        std::vector<std::string>{"robot"});

    // Effects
    this->add_effect(pddl::END, "carrying",
                     std::vector<std::string>{"robot", "item"});
    this->add_effect(pddl::START, "gripper_free",
                     std::vector<std::string>{"robot"}, true);
    this->add_effect(pddl::END, "item_at",
                     std::vector<std::string>{"item", "location"}, true);

    auto node = yasmin_ros::YasminNode::get_instance();
    this->action_client_ =
        rclcpp_action::create_client<ExecuteTaskAction>(node, "execute_action");
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    PROFILE_FUNCTION();

    this->cancelled_ = false;

    std::string robot = params[0];
    std::string item = params[1];
    std::string location = params[2];
    auto node = yasmin_ros::YasminNode::get_instance();

    if (!restaurant_utils::is_battery_ok(robot)) {
      auto node_tmp = yasmin_ros::YasminNode::get_instance();
      RCLCPP_WARN(node_tmp->get_logger(),
                  "[%s] has low battery, aborting pick.", robot.c_str());
      return pddl::ActionStatus::ABORTED;
    }

    if (!this->action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(node->get_logger(),
                   "[%s] execute_action server not available", robot.c_str());
      return pddl::ActionStatus::ABORTED;
    }

    auto goal = ExecuteTaskAction::Goal();
    goal.action.robot = robot;
    goal.action.type = "pick";
    goal.action.object = item;

    RCLCPP_INFO(node->get_logger(), "[%s] picking up %s from %s", robot.c_str(),
                item.c_str(), location.c_str());

    if (this->cancelled_) {
      RCLCPP_INFO(node->get_logger(), "[%s] Pick action cancelled.",
                  robot.c_str());
      return pddl::ActionStatus::CANCELED;
    }

    auto goal_handle_future = this->action_client_->async_send_goal(goal);

    // Wait for the goal to be accepted
    goal_handle_future.wait();
    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(),
                   "[%s] Goal was rejected by the action server.",
                   robot.c_str());
      return pddl::ActionStatus::ABORTED;
    }

    // Wait for the result while periodically checking for cancellation.
    auto result_future = this->action_client_->async_get_result(goal_handle);
    result_future.wait();

    // Check the result
    auto wrapped_result = result_future.get();
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { this->cancelled_ = true; }

private:
  rclcpp_action::Client<ExecuteTaskAction>::SharedPtr action_client_;
  std::atomic<bool> cancelled_{false};
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PickAction, omni_plan::pddl::Action)
