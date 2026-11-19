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

/// @brief Navigate between locations with explicit connectivity constraints.
/// PDDL: (navigate_connected ?robot ?from ?to)
class NavigateConnectedAction : public pddl::Action {
public:
  NavigateConnectedAction()
      : Action("navigate_connected", 20,
               {
                   {"robot", "robot"},
                   {"from", "location"},
                   {"to", "location"},
               }) {

    // Preconditions
    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "from"});
    this->add_condition(pddl::START, "battery_ok",
                        std::vector<std::string>{"robot"});
    this->add_condition(pddl::START, "connected",
                        std::vector<std::string>{"from", "to"});

    // Effects
    this->add_effect(pddl::START, "robot_at",
                     std::vector<std::string>{"robot", "from"}, true);
    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "to"});

    auto node = yasmin_ros::YasminNode::get_instance();
    this->action_client_ =
        rclcpp_action::create_client<ExecuteTaskAction>(node, "execute_action");
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    PROFILE_FUNCTION();

    this->cancelled_ = false;

    std::string robot = params[0];
    std::string from = params[1];
    std::string to = params[2];
    auto node = yasmin_ros::YasminNode::get_instance();

    if (!restaurant_utils::is_battery_ok(robot)) {
      RCLCPP_WARN(node->get_logger(),
                  "[%s] %s battery dropped during navigate, "
                  "aborting.",
                  robot.c_str(), robot.c_str());
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
    goal.action.type = "navigate";
    goal.action.target_location = to;

    RCLCPP_INFO(node->get_logger(), "[%s] navigating from %s to %s",
                robot.c_str(), from.c_str(), to.c_str());

    if (this->cancelled_) {
      RCLCPP_INFO(node->get_logger(), "[%s] Navigate action cancelled.",
                  robot.c_str());
      return pddl::ActionStatus::CANCELED;
    }

    auto goal_handle_future = this->action_client_->async_send_goal(goal);

    goal_handle_future.wait();
    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(),
                   "[%s] Goal was rejected by the action server.",
                   robot.c_str());
      return pddl::ActionStatus::ABORTED;
    }

    auto result_future = this->action_client_->async_get_result(goal_handle);
    result_future.wait();

    auto wrapped_result = result_future.get();
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { this->cancelled_ = true; }

private:
  rclcpp_action::Client<ExecuteTaskAction>::SharedPtr action_client_;
  std::atomic<bool> cancelled_{false};
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(NavigateConnectedAction, omni_plan::pddl::Action)
