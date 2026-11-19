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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "knowledge_graph/knowledge_graph.hpp"
#include "omni_plan_pyrobosim_msgs/msg/package_delivered.hpp"
#include "pyrobosim_msgs/srv/add_object.hpp"
#include "pyrobosim_msgs/srv/get_objects.hpp"
#include "pyrobosim_msgs/srv/remove_object.hpp"
#include "pyrobosim_msgs/srv/request_world_state.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

static const std::vector<std::string> ENTRANCE_TABLES = {"entrance_table1",
                                                         "entrance_table2"};
static const std::vector<std::string> SHELVES = {
    "storage_shelf1", "storage_shelf2", "storage_shelf3",
    "storage_shelf4", "storage_shelf5", "storage_shelf6"};
static const std::vector<std::string> PREP_TABLES = {
    "prep_table1", "prep_table2", "prep_table3"};
static const std::vector<std::string> DELIVERY_TABLES = {"delivery_table1",
                                                         "delivery_table2"};

constexpr static int MAX_PACKAGES_PER_TABLE = 4;
class WarehousePackageSim : public rclcpp::Node {

public:
  WarehousePackageSim() : Node("warehouse_package_sim") {
    // Publisher for package delivery reports
    this->package_delivered_pub_ =
        this->create_publisher<omni_plan_pyrobosim_msgs::msg::PackageDelivered>(
            "package_delivered", 10);
    // Parameters
    this->declare_parameter("package_arrival_min", 30.0);
    this->declare_parameter("package_arrival_max", 45.0);
    this->declare_parameter("storage_time_min", 60.0);
    this->declare_parameter("storage_time_max", 120.0);
    this->declare_parameter("prep_time_min", 5.0);
    this->declare_parameter("prep_time_max", 10.0);
    this->declare_parameter("removal_interval", 60.0);
    this->declare_parameter("goal_poll_interval", 2.0);

    this->arrival_min_ = this->get_parameter("package_arrival_min").as_double();
    this->arrival_max_ = this->get_parameter("package_arrival_max").as_double();
    this->storage_min_ = this->get_parameter("storage_time_min").as_double();
    this->storage_max_ = this->get_parameter("storage_time_max").as_double();
    this->prep_min_ = this->get_parameter("prep_time_min").as_double();
    this->prep_max_ = this->get_parameter("prep_time_max").as_double();
    this->removal_interval_ =
        this->get_parameter("removal_interval").as_double();
    this->poll_interval_ =
        this->get_parameter("goal_poll_interval").as_double();

    // Initialize spin thread
    this->executor_ =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->spin_thread_ = std::thread([this]() {
      this->executor_->add_node(this->get_node_base_interface());
      this->executor_->spin();
    });

    // Pyrobosim service clients
    this->req_world_state_cli_ =
        this->create_client<pyrobosim_msgs::srv::RequestWorldState>(
            "request_world_state");
    this->add_obj_cli_ =
        this->create_client<pyrobosim_msgs::srv::AddObject>("add_object");
    this->rm_obj_cli_ =
        this->create_client<pyrobosim_msgs::srv::RemoveObject>("remove_object");
    this->get_objs_cli_ =
        this->create_client<pyrobosim_msgs::srv::GetObjects>("get_objects");

    // Knowledge Graph
    this->kg_ = knowledge_graph::KnowledgeGraph::get_instance();
    this->initialize_knowledge_graph();

    RCLCPP_INFO(this->get_logger(), "Waiting for pyrobosim services ...");
    this->req_world_state_cli_->wait_for_service(std::chrono::seconds(60));
    this->add_obj_cli_->wait_for_service(std::chrono::seconds(60));
    this->rm_obj_cli_->wait_for_service(std::chrono::seconds(60));
    this->get_objs_cli_->wait_for_service(std::chrono::seconds(60));
    RCLCPP_INFO(this->get_logger(), "All pyrobosim services available.");

    // Start one thread per entrance table to spawn packages (not per package)
    for (const auto &t : ENTRANCE_TABLES) {
      this->table_threads_.emplace_back(
          std::thread(&WarehousePackageSim::package_spawner_loop, this, t));
    }

    // Removal thread
    this->removal_thread_ =
        std::thread(&WarehousePackageSim::removal_loop, this);

    RCLCPP_INFO(this->get_logger(), "Warehouse package simulation started.");
  }

  ~WarehousePackageSim() override {
    this->running_.store(false);
    for (auto &t : this->table_threads_) {
      if (t.joinable())
        t.join();
    }
    if (this->removal_thread_.joinable())
      this->removal_thread_.join();
    // Join all package threads
    {
      std::lock_guard<std::mutex> lock(this->package_threads_mutex_);
      for (auto &t : this->package_threads_) {
        if (t.joinable())
          t.join();
      }
    }
  }

  void join_spin() {
    if (this->spin_thread_.joinable())
      this->spin_thread_.join();
  }

private:
  // No longer need to track active packages per table with a counter
  // Instead, count packages at each table using the knowledge graph
  void initialize_knowledge_graph() {
    // Rooms
    this->kg_->create_node("warehouse", "warehouse");
    this->kg_->create_node("entrance", "room");
    this->kg_->create_node("package_preparation", "room");
    this->kg_->create_node("storage", "room");
    this->kg_->create_node("delivery", "room");

    // Locations
    for (const auto &t : ENTRANCE_TABLES)
      this->kg_->create_node(t, "location");
    for (const auto &s : SHELVES) {
      this->kg_->create_node(s, "location");
    }
    for (const auto &p : PREP_TABLES) {
      this->kg_->create_node(p, "location");
    }
    for (const auto &d : DELIVERY_TABLES) {
      this->kg_->create_node(d, "location");
    }
    // Chargers
    std::vector<std::string> chargers = {"charger1", "charger2", "charger3",
                                         "charger4"};
    for (const auto &c : chargers) {
      this->kg_->create_node(c, "location");
      this->kg_->create_edge("is_charger", c, c);
    }
    this->kg_->create_edge("in", "charger1", "entrance");
    this->kg_->create_edge("in", "charger2", "storage");
    this->kg_->create_edge("in", "charger3", "package_preparation");
    this->kg_->create_edge("in", "charger4", "delivery");

    this->kg_->create_node("tray1", "item");
    this->kg_->create_edge("item_at", "tray1", "storage_shelf6");

    // Room/Location edges
    this->kg_->create_edge("has", "warehouse", "entrance");
    this->kg_->create_edge("has", "warehouse", "package_preparation");
    this->kg_->create_edge("has", "warehouse", "storage");
    this->kg_->create_edge("has", "warehouse", "delivery");
    this->kg_->create_edge("in", "prep_table1", "package_preparation");
    this->kg_->create_edge("in", "prep_table2", "package_preparation");
    this->kg_->create_edge("in", "prep_table3", "package_preparation");
    for (const auto &t : ENTRANCE_TABLES)
      this->kg_->create_edge("in", t, "entrance");
    for (const auto &s : SHELVES)
      this->kg_->create_edge("in", s, "storage");
    for (const auto &d : DELIVERY_TABLES)
      this->kg_->create_edge("in", d, "delivery");

    // Print initial state
    std::cout << "Warehouse Demo - Knowledge Graph" << std::endl;
    std::cout << "============================================" << std::endl;
    auto nodes = this->kg_->get_nodes();
    std::cout << "Nodes (" << nodes.size() << "):" << std::endl;
    for (const auto &node : nodes) {
      std::cout << "  " << node.to_string() << std::endl;
    }
    auto edges = this->kg_->get_edges();
    std::cout << "Edges (" << edges.size() << "):" << std::endl;
    for (const auto &edge : edges) {
      std::cout << "  " << edge.to_string() << std::endl;
    }

    // ── Robot state ──
    RCLCPP_INFO(this->get_logger(),
                "Querying initial world state from pyrobosim ...");
    auto req =
        std::make_shared<pyrobosim_msgs::srv::RequestWorldState::Request>();
    auto resp = this->call_service(this->req_world_state_cli_, req, 60.0);

    while (resp == nullptr) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to get initial world state from pyrobosim. Retrying ...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      resp = this->call_service(this->req_world_state_cli_, req, 60.0);
    }
    RCLCPP_INFO(this->get_logger(), "Received world state from pyrobosim.");

    for (auto const &robot : resp->state.robots) {
      RCLCPP_INFO(this->get_logger(), "  Robot: %s at %s", robot.name.c_str(),
                  robot.last_visited_location.c_str());
      std::string robot_name = robot.name;
      std::string location = robot.last_visited_location;

      // Remove last _<name> suffix from location if present
      size_t last_underscore = location.find_last_of('_');
      if (last_underscore != std::string::npos) {
        location = location.substr(0, last_underscore);
      }

      this->kg_->create_node(robot_name, "robot");
      this->kg_->create_edge("robot_at", robot_name, location);
      this->kg_->create_edge("gripper_free", robot_name, robot_name);
      this->kg_->create_edge("battery_ok", robot_name, robot_name);
    }
    std::cout << "\nInitial state loaded. Starting package simulation ..."
              << std::endl;
  }

  // Spawner loop: for each entrance table, spawn a new thread for each package
  void package_spawner_loop(const std::string &table) {
    std::mt19937 rng(std::random_device{}());
    while (this->running_.load() && rclcpp::ok()) {
      // Count number of packages currently at this entrance table
      int num_packages = 0;
      auto edges = this->kg_->get_edges();
      for (const auto &edge : edges) {
        if (edge.get_type() == "item_at" && edge.get_target_node() == table) {
          num_packages++;
        }
      }

      if (num_packages >= MAX_PACKAGES_PER_TABLE) {
        RCLCPP_INFO(this->get_logger(),
                    "[%s] Max packages (%d) reached, waiting ...",
                    table.c_str(), MAX_PACKAGES_PER_TABLE);
        this->sleep_interruptible(this->poll_interval_);
        continue;
      }

      std::uniform_real_distribution<double> delay_dist(this->arrival_min_,
                                                        this->arrival_max_);
      double delay = delay_dist(rng);
      RCLCPP_INFO(this->get_logger(),
                  "[%s] Next package arriving in %.1f s ...", table.c_str(),
                  delay);
      this->sleep_interruptible(delay);
      if (!this->running_.load() || !rclcpp::ok())
        continue;

      // Spawn a new thread for each package
      {
        std::lock_guard<std::mutex> lock(this->package_threads_mutex_);
        this->package_threads_.emplace_back(
            std::thread(&WarehousePackageSim::handle_package_lifecycle_thread,
                        this, table));
      }
    }
  }

  // Thread entry point for a single package lifecycle
  void handle_package_lifecycle_thread(const std::string &table) {
    std::mt19937 rng(std::random_device{}());
    this->handle_package_lifecycle(table, rng);
  }

  void handle_package_lifecycle(const std::string &table, std::mt19937 &rng) {
    int package_id;
    {
      std::lock_guard<std::mutex> lock(this->counter_mutex_);
      package_id = ++this->package_counter_;
    }

    std::string package_name = "package" + std::to_string(package_id);
    std::string category = "package";
    auto start_time_steady = std::chrono::steady_clock::now();
    auto start_time_ros = this->now();

    // Spawn package at entrance table
    auto req = std::make_shared<pyrobosim_msgs::srv::AddObject::Request>();
    req->category = category;
    req->name = package_name;
    req->parent = table;
    auto resp = this->call_service(this->add_obj_cli_, req);

    while (!resp || !resp->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn %s at %s. Retrying ...",
                   package_name.c_str(), table.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
      resp = this->call_service(this->add_obj_cli_, req);
    }

    this->kg_->create_node(package_name, "item");
    this->kg_->create_edge("item_at", package_name, table);
    RCLCPP_INFO(this->get_logger(),
                "[%s] Package %s spawned at entrance table.", table.c_str(),
                package_name.c_str());

    // Set goal for storing
    std::uniform_int_distribution<size_t> shelf_dist(0, SHELVES.size() - 1);
    std::string shelf = SHELVES[shelf_dist(rng)];
    auto shelf_goal = this->kg_->create_edge("item_at", package_name, shelf);
    shelf_goal.set_property("is_goal", true);
    this->kg_->update_edge(shelf_goal);

    // Wait for robot to move the item (goal fulfilled: item_at at shelf)
    this->wait_for_edge_fulfilled("item_at", package_name, shelf);

    // Simulate time before preparation is needed
    std::uniform_real_distribution<double> storage_delay(this->storage_min_,
                                                         this->storage_max_);
    double storage_time = storage_delay(rng);
    this->sleep_interruptible(storage_time);

    // Set goal for preparing the package at a prep table
    std::uniform_int_distribution<size_t> prep_dist(0, PREP_TABLES.size() - 1);
    std::string prep_table = PREP_TABLES[prep_dist(rng)];
    auto prep_location_goal =
        this->kg_->create_edge("item_at", package_name, prep_table);
    prep_location_goal.set_property("is_goal", true);
    this->kg_->update_edge(prep_location_goal);

    // Wait for robot to fulfill the preparation goal
    this->wait_for_edge_fulfilled("item_at", package_name, prep_table);

    // Simulate time after preparation is needed
    std::uniform_real_distribution<double> prep_delay(this->prep_min_,
                                                      this->prep_max_);
    double prep_time = prep_delay(rng);
    this->sleep_interruptible(prep_time);

    // Set goal for delivery
    std::uniform_int_distribution<size_t> delivery_dist(
        0, DELIVERY_TABLES.size() - 1);
    std::string delivery_table = DELIVERY_TABLES[delivery_dist(rng)];
    auto delivery_location_goal =
        this->kg_->create_edge("item_at", package_name, delivery_table);
    delivery_location_goal.set_property("is_goal", true);
    this->kg_->update_edge(delivery_location_goal);

    // Wait for robot to fulfill the delivery goal
    this->wait_for_edge_fulfilled("item_at", package_name, delivery_table);

    // Publish PackageDelivered message
    auto end_time_ros = this->now();
    auto end_time_steady = std::chrono::steady_clock::now();
    double total_duration_sec =
        std::chrono::duration<double>(end_time_steady - start_time_steady)
            .count();

    omni_plan_pyrobosim_msgs::msg::PackageDelivered msg;
    msg.package_id = package_id;
    msg.total_duration_sec = total_duration_sec;
    msg.start_time = start_time_ros;
    msg.end_time = end_time_ros;
    this->package_delivered_pub_->publish(msg);
  }

  void wait_for_edge_fulfilled(const std::string &type,
                               const std::string &source,
                               const std::string &target) {
    while (this->running_.load() && rclcpp::ok()) {
      if (this->kg_->has_edge(type, source, target)) {
        try {
          auto edge = this->kg_->get_edge(type, source, target);
          if (!edge.has_property("is_goal")) {
            return;
          } else {
            if (!edge.get_property<bool>("is_goal")) {
              return;
            }
          }
        } catch (const std::runtime_error &e) {
          // Edge might have been removed, treat as fulfilled
          return;
        }
      }
      this->sleep_interruptible(this->poll_interval_);
    }
  }

  void removal_loop() {

    while (this->running_.load() && rclcpp::ok()) {
      this->sleep_interruptible(this->removal_interval_);

      // Remove all packages at delivery tables
      for (const auto &d : DELIVERY_TABLES) {
        auto edges = this->kg_->get_edges();
        for (const auto &edge : edges) {
          if (edge.get_type() == "item_at" && edge.get_target_node() == d) {

            if (edge.has_property("is_goal")) {
              if (edge.get_property<bool>("is_goal")) {
                continue;
              }
            }

            // Remove from pyrobosim
            std::string item = edge.get_source_node();
            auto rm_req =
                std::make_shared<pyrobosim_msgs::srv::RemoveObject::Request>();
            rm_req->name = item;
            this->call_service(this->rm_obj_cli_, rm_req);

            // Remove from KG
            this->kg_->remove_edge(edge);
            if (this->kg_->has_node(item)) {
              this->kg_->remove_node(this->kg_->get_node(item));
            }
            RCLCPP_INFO(this->get_logger(),
                        "Removed delivered package %s from %s.", item.c_str(),
                        d.c_str());
          }
        }
      }
    }
  }

  void sleep_interruptible(double seconds) {
    auto end = std::chrono::steady_clock::now() +
               std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end && this->running_.load() &&
           rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr
  call_service(std::shared_ptr<rclcpp::Client<ServiceT>> client,
               std::shared_ptr<typename ServiceT::Request> request,
               double timeout_sec = 10.0) {
    client->wait_for_service(std::chrono::duration<double>(timeout_sec));
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(timeout_sec)) ==
        std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedPtr
      req_world_state_cli_;
  rclcpp::Client<pyrobosim_msgs::srv::AddObject>::SharedPtr add_obj_cli_;
  rclcpp::Client<pyrobosim_msgs::srv::RemoveObject>::SharedPtr rm_obj_cli_;
  rclcpp::Client<pyrobosim_msgs::srv::GetObjects>::SharedPtr get_objs_cli_;
  rclcpp::Publisher<omni_plan_pyrobosim_msgs::msg::PackageDelivered>::SharedPtr
      package_delivered_pub_;

  double arrival_min_;
  double arrival_max_;
  double storage_min_;
  double storage_max_;
  double prep_min_;
  double prep_max_;
  double removal_interval_;
  double poll_interval_;
  int package_counter_ = 0;
  std::mutex counter_mutex_;
  std::vector<std::thread> table_threads_;
  std::vector<std::thread> package_threads_;
  std::mutex package_threads_mutex_;
  std::thread removal_thread_;
  std::thread spin_thread_;
  rclcpp::Executor::SharedPtr executor_;
  std::atomic<bool> running_{true};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WarehousePackageSim>();
  node->join_spin();
  rclcpp::shutdown();
  return 0;
}
