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

#include <chrono>
#include <string>

#include "knowledge_graph/knowledge_graph.hpp"
#include "pyrobosim_msgs/srv/request_world_state.hpp"
#include "rclcpp/rclcpp.hpp"

class BatteryMonitor : public rclcpp::Node {
public:
  BatteryMonitor() : Node("battery_monitor") {
    this->declare_parameter("check_interval", 0.05);
    this->declare_parameter("low_battery_threshold", 50.0);

    auto check_interval = this->get_parameter("check_interval").as_double();
    this->low_threshold_ =
        this->get_parameter("low_battery_threshold").as_double();

    this->kg_ = knowledge_graph::KnowledgeGraph::get_instance();

    this->world_state_cli_ =
        this->create_client<pyrobosim_msgs::srv::RequestWorldState>(
            "request_world_state");

    RCLCPP_INFO(this->get_logger(),
                "Waiting for /request_world_state service ...");
    this->world_state_cli_->wait_for_service(std::chrono::seconds(60));
    RCLCPP_INFO(this->get_logger(),
                "Battery monitor started (interval=%.1f s, threshold=%.1f%%).",
                check_interval, this->low_threshold_);

    // Sleep briefly to allow other nodes to initialize and populate the KG
    rclcpp::sleep_for(std::chrono::seconds(5));

    // Create wall timer for periodic battery checks
    this->timer_ =
        this->create_wall_timer(std::chrono::duration<double>(check_interval),
                                [this]() { this->timer_callback(); });
  }

  ~BatteryMonitor() override {
    if (this->timer_) {
      this->timer_->cancel();
    }
  }

private:
  void timer_callback() {
    // Only send a new request if the previous one has completed or timed out
    if (this->response_future_.valid()) {
      auto status = this->response_future_.wait_for(std::chrono::seconds(0));
      if (status != std::future_status::ready) {
        RCLCPP_DEBUG(this->get_logger(),
                     "[battery_monitor] Previous request still pending.");
        return;
      }
      // Previous request completed; retrieve and discard result
      try {
        this->response_future_.get();
      } catch (...) {
      }
    }

    // Send new async request
    auto request =
        std::make_shared<pyrobosim_msgs::srv::RequestWorldState::Request>();

    auto response_received_callback =
        [this](typename rclcpp::Client<
               pyrobosim_msgs::srv::RequestWorldState>::SharedFuture future) {
          this->response_callback(future);
        };

    this->world_state_cli_->wait_for_service(std::chrono::seconds(60));
    auto result = this->world_state_cli_->async_send_request(
        request, response_received_callback);
    this->response_future_ = result.future;
  }

  void response_callback(
      rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedFuture
          future) {
    try {
      auto response = future.get();
      for (const auto &robot : response->state.robots) {
        this->update_battery_kg(robot.name,
                                static_cast<double>(robot.battery_level));
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "[battery_monitor] Service call failed: %s", e.what());
    }
  }

  void update_battery_kg(const std::string &robot, double battery_level) {
    bool low = battery_level < this->low_threshold_;
    RCLCPP_INFO(this->get_logger(),
                "[battery_monitor] %s battery level: %.1f%% (%s)",
                robot.c_str(), battery_level, low ? "LOW" : "OK");

    try {
      // Create the edge if this instance does not have it (e.g. the initial
      // graph sync missed it). Without this, a low battery would never be
      // turned into a goal and the robot would replan the same action
      // forever.
      if (!this->kg_->has_edge("battery_ok", robot, robot)) {
        this->kg_->create_edge("battery_ok", robot, robot);
      }

      auto edge = this->kg_->get_edge("battery_ok", robot, robot);
      bool is_goal =
          edge.has_property("is_goal") && edge.get_property<bool>("is_goal");

      if (low && !is_goal) {
        edge.set_property<bool>("is_goal", true);
        this->kg_->update_edge(edge);
        RCLCPP_INFO(this->get_logger(),
                    "[battery_monitor] %s battery LOW: %.1f%% "
                    "(added goal property)",
                    robot.c_str(), battery_level);
      } else if (!low && is_goal) {
        // Clear a stale charge goal once the battery is healthy again
        // (e.g. left behind after charging), otherwise the planner would
        // keep inserting charge cycles into every plan.
        edge.set_property<bool>("is_goal", false);
        this->kg_->update_edge(edge);
        RCLCPP_INFO(this->get_logger(),
                    "[battery_monitor] %s battery OK: %.1f%% "
                    "(removed goal property)",
                    robot.c_str(), battery_level);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "[battery_monitor] Could not update edge for %s: %s",
                  robot.c_str(), e.what());
    }
  }

  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedPtr
      world_state_cli_;

  double low_threshold_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedFuture
      response_future_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
