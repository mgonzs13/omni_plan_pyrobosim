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
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "battery_utils.hpp"
#include "omni_plan/pddl/action.hpp"
#include "poirot/poirot.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace omni_plan;

/// @brief Take the order from a table and relay it to the kitchen.
/// The waiter must be at the table. After taking the order, an
/// "order_ready" fact edge is created (table → kitchen_counter).
/// PDDL: (take_order ?robot ?table ?kitchen)
class TakeOrderAction : public pddl::Action {
public:
  TakeOrderAction()
      : Action("take_order", 3,
               {
                   {"robot", "robot"},
                   {"table", "location"},
                   {"kitchen", "location"},
               }) {

    // Preconditions
    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "table"});
    this->add_condition(pddl::START, "order_pending",
                        std::vector<std::string>{"table"});
    this->add_condition(pddl::START, "battery_ok",
                        std::vector<std::string>{"robot"});

    // Effects
    this->add_effect(pddl::END, "order_pending",
                     std::vector<std::string>{"table"}, true);
    this->add_effect(pddl::END, "order_ready",
                     std::vector<std::string>{"table", "kitchen"});
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    PROFILE_FUNCTION();

    this->cancelled_ = false;

    std::string robot = params[0];
    std::string table = params[1];
    std::string kitchen = params[2];
    auto node = yasmin_ros::YasminNode::get_instance();

    if (!restaurant_utils::is_battery_ok(robot)) {
      auto node_tmp = yasmin_ros::YasminNode::get_instance();
      RCLCPP_WARN(node_tmp->get_logger(),
                  "[%s] has low battery, aborting pick.", robot.c_str());
      return pddl::ActionStatus::ABORTED;
    }

    RCLCPP_INFO(node->get_logger(), "[%s] taking order at %s for %s",
                robot.c_str(), table.c_str(), kitchen.c_str());

    if (this->cancelled_) {
      RCLCPP_INFO(node->get_logger(),
                  "[%s] Take-order action cancelled for %s.", robot.c_str(),
                  robot.c_str());
      return pddl::ActionStatus::CANCELED;
    }

    // Sleep for 3 seconds to simulate the time taken to take the order
    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(node->get_logger(), "[%s] relayed order from %s to %s",
                robot.c_str(), table.c_str(), kitchen.c_str());
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { this->cancelled_ = true; }

private:
  std::atomic<bool> cancelled_{false};
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TakeOrderAction, omni_plan::pddl::Action)
