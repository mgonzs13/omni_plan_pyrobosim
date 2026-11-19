// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

/// @brief Place a carried item on a table/counter in the pyrobosim restaurant.
/// PDDL: (place ?robot ?item ?location)
class PlaceAction : public pddl::Action {
public:
  PlaceAction()
      : Action("place", 5,
               {
                   {"robot", "robot"},
                   {"item", "item"},
                   {"location", "location"},
               }) {

    // Preconditions
    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "location"});
    this->add_condition(pddl::START, "carrying",
                        std::vector<std::string>{"robot", "item"});
    this->add_condition(pddl::START, "battery_ok",
                        std::vector<std::string>{"robot"});

    // Effects
    this->add_effect(pddl::END, "carrying",
                     std::vector<std::string>{"robot", "item"}, true);
    this->add_effect(pddl::END, "gripper_free",
                     std::vector<std::string>{"robot"});
    this->add_effect(pddl::END, "item_at",
                     std::vector<std::string>{"item", "location"});

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
    goal.action.type = "place";
    goal.action.object = item;
    goal.action.target_location = location;

    RCLCPP_INFO(node->get_logger(), "[%s] placing %s at %s", robot.c_str(),
                item.c_str(), location.c_str());

    if (this->cancelled_) {
      RCLCPP_INFO(node->get_logger(), "[%s] Place action cancelled.",
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
PLUGINLIB_EXPORT_CLASS(PlaceAction, omni_plan::pddl::Action)
