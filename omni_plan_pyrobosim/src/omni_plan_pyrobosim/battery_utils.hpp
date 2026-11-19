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

#pragma once

/// @file battery_utils.hpp
/// @brief Shared battery query utilities for the pyrobosim restaurant demo.
///
/// Provides helpers to query robot battery levels via the
/// /request_world_state ROS 2 service. Used by both action plugins and the
/// battery monitor node.

#include <chrono>
#include <optional>
#include <string>

#include "pyrobosim_msgs/srv/request_world_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace restaurant_utils {

/// Battery level below which a robot is considered low (percentage).
static constexpr double BATTERY_LOW_THRESHOLD = 50.0;

/// @brief Query the battery level of a named robot via /request_world_state.
///
/// Uses a single shared service client on the YasminNode (created on first
/// use). Blocks the calling thread for up to 5 seconds waiting for the
/// service and the response.
///
/// @param robot_name  Name of the robot (e.g. "waiter1").
/// @return Battery level in [0.0, 100.0], or std::nullopt on failure.
inline std::optional<double>
get_robot_battery_level(const std::string &robot_name) {
  auto node = yasmin_ros::YasminNode::get_instance();

  static auto client =
      node->create_client<pyrobosim_msgs::srv::RequestWorldState>(
          "request_world_state");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(node->get_logger(),
                "[battery_utils] /request_world_state not available");
    return std::nullopt;
  }

  auto request =
      std::make_shared<pyrobosim_msgs::srv::RequestWorldState::Request>();

  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_WARN(node->get_logger(),
                "[battery_utils] /request_world_state timed out for %s",
                robot_name.c_str());
    return std::nullopt;
  }

  auto response = future.get();
  if (!response->state.robots.empty()) {
    for (const auto &robot : response->state.robots) {
      if (robot.name == robot_name) {
        return static_cast<double>(robot.battery_level);
      }
    }
  }
  return std::nullopt;
}

/// @brief Returns true when the robot's battery is at or above the low
/// threshold. Call this at the start of an action's run() to abort early if
/// the battery has dropped since the plan was generated.
///
/// A failure to query the battery (service timeout, unavailable) does NOT
/// abort the action: the plan already checked the battery_ok precondition,
/// and an aborted action leaves the knowledge unchanged, which would make
/// the planner regenerate the exact same plan forever.
///
/// @param robot_name  Name of the robot.
/// @return true if battery >= BATTERY_LOW_THRESHOLD (or unknown), false
/// otherwise.
inline bool is_battery_ok(const std::string &robot_name) {
  auto level = get_robot_battery_level(robot_name);
  if (!level.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("battery_utils"),
                "[%s] battery level unknown, proceeding", robot_name.c_str());
    return true;
  }
  return level.value() >= BATTERY_LOW_THRESHOLD;
}

} // namespace restaurant_utils
