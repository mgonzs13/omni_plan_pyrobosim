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

#include "omni_plan_knowledge_base/knowledge_base_client.hpp"
#include "pyrobosim_msgs/srv/request_world_state.hpp"
#include "rclcpp/rclcpp.hpp"

class BatteryMonitorKb : public rclcpp::Node {
public:
  BatteryMonitorKb() : Node("battery_monitor_kb") {
    this->declare_parameter("check_interval", 0.05);
    this->declare_parameter("low_battery_threshold", 50.0);

    auto check_interval = this->get_parameter("check_interval").as_double();
    this->low_threshold_ =
        this->get_parameter("low_battery_threshold").as_double();

    this->kb_ = std::make_shared<omni_plan_knowledge_base::KnowledgeBaseClient>(
        "battery_monitor_kb_client");

    this->world_state_cli_ =
        this->create_client<pyrobosim_msgs::srv::RequestWorldState>(
            "request_world_state");

    RCLCPP_INFO(this->get_logger(),
                "Waiting for /request_world_state service ...");
    this->world_state_cli_->wait_for_service(std::chrono::seconds(60));
    RCLCPP_INFO(this->get_logger(),
                "Battery monitor (KB) started (interval=%.1f s, "
                "threshold=%.1f%%).",
                check_interval, this->low_threshold_);

    rclcpp::sleep_for(std::chrono::seconds(5));

    this->timer_ =
        this->create_wall_timer(std::chrono::duration<double>(check_interval),
                                [this]() { this->timer_callback(); });
  }

  ~BatteryMonitorKb() override {
    if (this->timer_) {
      this->timer_->cancel();
    }
  }

private:
  bool has_goal(const std::string &name,
                const std::vector<std::string> &args) const {
    auto goals = this->kb_->get_goals();
    for (const auto &g : goals) {
      if (g.get_name() == name && g.get_args() == args)
        return true;
    }
    return false;
  }

  bool has_fact(const std::string &name,
                const std::vector<std::string> &args) const {
    auto facts = this->kb_->get_facts(name);
    for (const auto &f : facts) {
      if (f.get_args() == args)
        return true;
    }
    return false;
  }

  void timer_callback() {
    if (this->response_future_.valid()) {
      auto status = this->response_future_.wait_for(std::chrono::seconds(0));
      if (status != std::future_status::ready) {
        RCLCPP_DEBUG(this->get_logger(),
                     "[battery_monitor_kb] Previous request still pending.");
        return;
      }
      try {
        this->response_future_.get();
      } catch (...) {
      }
    }

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
        this->update_battery_kb(robot.name,
                                static_cast<double>(robot.battery_level));
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "[battery_monitor_kb] Service call failed: %s", e.what());
    }
  }

  void update_battery_kb(const std::string &robot, double battery_level) {
    bool low = battery_level < this->low_threshold_;
    RCLCPP_INFO(this->get_logger(),
                "[battery_monitor_kb] %s battery level: %.1f%% (%s)",
                robot.c_str(), battery_level, low ? "LOW" : "OK");

    if (low) {
      // Ensure the charge goal is present even if the battery_ok fact is
      // missing (e.g. the KB was not fully initialised yet), otherwise a
      // low battery would never trigger a charge plan and the robot would
      // replan the same failing action forever.
      if (!this->has_goal("battery_ok", {robot})) {
        if (this->has_fact("battery_ok", {robot})) {
          this->kb_->remove_fact("battery_ok", {robot});
        }
        this->kb_->add_goal("battery_ok", {robot});
        RCLCPP_INFO(this->get_logger(),
                    "[battery_monitor_kb] %s battery LOW: %.1f%% "
                    "(added goal)",
                    robot.c_str(), battery_level);
      }
    } else {
      // Clear a stale charge goal once the battery is healthy again,
      // otherwise the planner would keep inserting charge cycles into every
      // plan.
      if (this->has_goal("battery_ok", {robot})) {
        this->kb_->remove_goal("battery_ok", {robot});
        if (!this->has_fact("battery_ok", {robot})) {
          this->kb_->add_fact("battery_ok", {robot});
        }
        RCLCPP_INFO(this->get_logger(),
                    "[battery_monitor_kb] %s battery OK: %.1f%% "
                    "(removed goal)",
                    robot.c_str(), battery_level);
      }
    }
  }

  std::shared_ptr<omni_plan_knowledge_base::KnowledgeBaseClient> kb_;
  rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedPtr
      world_state_cli_;

  double low_threshold_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedFuture
      response_future_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryMonitorKb>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
