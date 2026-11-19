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

#include "omni_plan/pddl/action.hpp"
#include "poirot/poirot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace omni_plan;

/// @brief Charge the waiter robot's battery at a charger location.
///
/// The robot must already be navigated to the charger before this action runs
/// (handled by the navigate action).
class ChargeAction : public pddl::Action {
public:
  ChargeAction()
      : Action("charge", 60,
               {
                   {"robot", "robot"},
                   {"charger", "location"},
               }) {

    // Preconditions
    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "charger"});
    this->add_condition(pddl::START, "is_charger",
                        std::vector<std::string>{"charger"});

    // Effects
    this->add_effect(pddl::END, "battery_ok",
                     std::vector<std::string>{"robot"});
  }

  pddl::ActionStatus run(const std::vector<std::string> &params) override {
    PROFILE_FUNCTION();

    this->cancelled_ = false;

    std::string robot = params[0];
    std::string charger = params[1];

    auto node = yasmin_ros::YasminNode::get_instance();

    RCLCPP_INFO(node->get_logger(),
                "[%s] charging at %s waiting for full battery ...",
                robot.c_str(), charger.c_str());

    if (this->cancelled_) {
      RCLCPP_INFO(node->get_logger(), "[%s] Charge action cancelled for %s.",
                  robot.c_str(), robot.c_str());
      return pddl::ActionStatus::CANCELED;
    }

    // Sleep for 5 seconds to simulate the time taken to charge
    std::this_thread::sleep_for(std::chrono::seconds(5));

    RCLCPP_INFO(node->get_logger(), "[%s] Charge completed at %s.",
                robot.c_str(), charger.c_str());
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { this->cancelled_ = true; }

private:
  std::atomic<bool> cancelled_{false};
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ChargeAction, omni_plan::pddl::Action)
