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

/// @file restaurant_customer_sim.cpp
/// @brief Restaurant Customer Simulator
///
/// Continuously generates customer groups for the pyrobosim restaurant demo.
///
/// Each table runs its own thread. Lifecycle per table:
///   1. FREE      → a group of 1–4 customers arrives after a random delay
///   2. ORDERING  → an "order_ready" goal edge is created for the table
///   3. SERVING   → food & drinks spawned in pyrobosim; KG delivery goals set
///   4. EATING    → wait for all delivery goals, then eat for a random duration
///   5. CLEANING  → all dishes and drinks removed; one waste item spawned;
///                   cleanup goal set for that single waste item
///   6. FREE      → waste goals fulfilled, table is free again (loop)
///
/// A separate dumpster-emptying timer removes waste objects from the dumpster
/// periodically.

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
#include "omni_plan_pyrobosim_msgs/msg/order_completed.hpp"
#include "pyrobosim_msgs/srv/add_object.hpp"
#include "pyrobosim_msgs/srv/get_objects.hpp"
#include "pyrobosim_msgs/srv/remove_object.hpp"
#include "pyrobosim_msgs/srv/request_world_state.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

static const std::vector<std::string> TABLES = {"table1", "table2", "table3",
                                                "table4", "table5", "table6"};

// ── Table state
// ───────────────────────────────────────────────────────────────
enum class TableState { FREE, ORDERING, SERVING, EATING, CLEANING };

// ── Order item descriptor
// ─────────────────────────────────────────────────────

struct OrderItem {
  std::string name;     // e.g. "order3_pasta_dish"
  std::string category; // e.g. "pasta_dish"
  std::string spawn_location;
};

// ── Customer Simulator
// ────────────────────────────────────────────────────────
class RestaurantCustomerSimConnected : public rclcpp::Node {
public:
  RestaurantCustomerSimConnected() : Node("restaurant_customer_sim_connected") {

    // Parameters
    this->declare_parameter("customer_arrival_min", 15.0);
    this->declare_parameter("customer_arrival_max", 45.0);
    this->declare_parameter("eating_time_min", 10.0);
    this->declare_parameter("eating_time_max", 30.0);
    this->declare_parameter("order_preparation_min", 5.0);
    this->declare_parameter("order_preparation_max", 15.0);
    this->declare_parameter("goal_poll_interval", 2.0);
    this->declare_parameter("dumpster_empty_interval", 60.0);
    this->declare_parameter("max_group_size", 4);

    this->arrival_min_ =
        this->get_parameter("customer_arrival_min").as_double();
    this->arrival_max_ =
        this->get_parameter("customer_arrival_max").as_double();
    this->eating_min_ = this->get_parameter("eating_time_min").as_double();
    this->eating_max_ = this->get_parameter("eating_time_max").as_double();
    this->order_prep_min_ =
        this->get_parameter("order_preparation_min").as_double();
    this->order_prep_max_ =
        this->get_parameter("order_preparation_max").as_double();
    this->poll_interval_ =
        this->get_parameter("goal_poll_interval").as_double();
    this->dumpster_empty_interval_ =
        this->get_parameter("dumpster_empty_interval").as_double();
    this->max_group_size_ = this->get_parameter("max_group_size").as_int();

    // Initialize spin thread
    this->executor_ =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->spin_thread_ = std::thread([this]() {
      this->executor_->add_node(this->get_node_base_interface());
      this->spin_ready_.store(true);
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

    // Publisher for order completion reports
    this->order_completed_pub_ =
        this->create_publisher<omni_plan_pyrobosim_msgs::msg::OrderCompleted>(
            "order_completed", 10);

    RCLCPP_INFO(this->get_logger(), "Waiting for pyrobosim services ...");
    this->req_world_state_cli_->wait_for_service(std::chrono::seconds(60));
    this->add_obj_cli_->wait_for_service(std::chrono::seconds(60));
    this->rm_obj_cli_->wait_for_service(std::chrono::seconds(60));
    this->get_objs_cli_->wait_for_service(std::chrono::seconds(60));
    RCLCPP_INFO(this->get_logger(), "All pyrobosim services available.");

    // Per-table state
    for (const auto &t : TABLES) {
      this->table_states_[t] = TableState::FREE;
    }

    // Start one thread per table
    for (const auto &t : TABLES) {
      this->table_threads_.emplace_back(
          std::thread(&RestaurantCustomerSimConnected::table_loop, this, t));
    }

    // Dumpster-emptying thread
    this->dumpster_thread_ =
        std::thread(&RestaurantCustomerSimConnected::dumpster_loop, this);

    RCLCPP_INFO(this->get_logger(), "Customer simulation started.");
  }

  ~RestaurantCustomerSimConnected() override {
    this->running_.store(false);
    for (auto &t : this->table_threads_) {
      if (t.joinable())
        t.join();
    }
    if (this->dumpster_thread_.joinable())
      this->dumpster_thread_.join();
  }

  void join_spin() {
    if (this->spin_thread_.joinable())
      this->spin_thread_.join();
  }

private:
  // ── KG world initialization ──────────────────────────────────────────────
  void initialize_knowledge_graph() {
    // ── Rooms ──
    this->kg_->create_node("restaurant", "restaurant");
    this->kg_->create_node("entrance", "room");
    this->kg_->create_node("dining_room", "room");
    this->kg_->create_node("kitchen", "room");
    this->kg_->create_node("bar", "room");
    this->kg_->create_node("trash_room", "room");

    // ── Locations ──
    this->kg_->create_node("reception", "location");
    this->kg_->create_node("kitchen_counter", "location");
    this->kg_->create_node("bar_counter0", "location");
    this->kg_->create_node("dumpster", "location");
    this->kg_->create_node("charger1", "location");
    this->kg_->create_node("charger2", "location");
    this->kg_->create_node("charger3", "location");
    this->kg_->create_node("charger4", "location");
    this->kg_->create_node("charger5", "location");

    for (int i = 1; i <= 6; ++i) {
      this->kg_->create_node("table" + std::to_string(i), "location");
    }

    // ── Rooms/Locations ──
    this->kg_->create_node("tray1", "item");
    this->kg_->create_edge("item_at", "tray1", "reception");

    // ── Rooms/Locations ──
    this->kg_->create_edge("has", "restaurant", "entrance");
    this->kg_->create_edge("has", "restaurant", "dining_room");
    this->kg_->create_edge("has", "restaurant", "kitchen");
    this->kg_->create_edge("has", "restaurant", "bar");
    this->kg_->create_edge("has", "restaurant", "trash_room");

    this->kg_->create_edge("in", "reception", "entrance");
    this->kg_->create_edge("in", "kitchen_counter", "kitchen");
    this->kg_->create_edge("in", "dumpster", "trash_room");
    this->kg_->create_edge("in", "bar_counter0", "bar");
    this->kg_->create_edge("in", "charger1", "kitchen");
    this->kg_->create_edge("in", "charger2", "bar");
    this->kg_->create_edge("in", "charger3", "trash_room");
    this->kg_->create_edge("in", "charger4", "dining_room");
    this->kg_->create_edge("in", "charger5", "entrance");

    this->kg_->create_edge("is_charger", "charger1", "charger1");
    this->kg_->create_edge("is_charger", "charger2", "charger2");
    this->kg_->create_edge("is_charger", "charger3", "charger3");
    this->kg_->create_edge("is_charger", "charger4", "charger4");
    this->kg_->create_edge("is_charger", "charger5", "charger5");

    for (int i = 1; i <= 6; ++i) {
      this->kg_->create_edge("in", "table" + std::to_string(i), "dining_room");
    }

    // ── Connected edges (within dining_room) ──
    // All tables and charger4 are in dining_room
    std::vector<std::string> dining_locs = {
        "table1", "table2", "table3", "table4", "table5", "table6", "charger4"};
    for (size_t i = 0; i < dining_locs.size(); ++i) {
      for (size_t j = i + 1; j < dining_locs.size(); ++j) {
        this->kg_->create_edge("connected", dining_locs[i], dining_locs[j]);
        this->kg_->create_edge("connected", dining_locs[j], dining_locs[i]);
      }
    }

    // ── Connected edges (within entrance) ──
    this->kg_->create_edge("connected", "reception", "charger5");
    this->kg_->create_edge("connected", "charger5", "reception");

    // ── Connected edges (within kitchen) ──
    this->kg_->create_edge("connected", "kitchen_counter", "charger1");
    this->kg_->create_edge("connected", "charger1", "kitchen_counter");

    // ── Connected edges (within bar) ──
    this->kg_->create_edge("connected", "bar_counter0", "charger2");
    this->kg_->create_edge("connected", "charger2", "bar_counter0");

    // ── Connected edges (within trash_room) ──
    this->kg_->create_edge("connected", "dumpster", "charger3");
    this->kg_->create_edge("connected", "charger3", "dumpster");

    // ── Cross-room connected edges ──
    // Kitchen <-> Dining room (hallway exists)
    for (const auto &loc : dining_locs) {
      this->kg_->create_edge("connected", "kitchen_counter", loc);
      this->kg_->create_edge("connected", loc, "kitchen_counter");
    }

    // Bar <-> Dining room (hallway exists)
    for (const auto &loc : dining_locs) {
      this->kg_->create_edge("connected", "bar_counter0", loc);
      this->kg_->create_edge("connected", loc, "bar_counter0");
    }

    // Dining room <-> Entrance (hallway exists)
    for (const auto &loc : dining_locs) {
      this->kg_->create_edge("connected", "reception", loc);
      this->kg_->create_edge("connected", loc, "reception");
    }

    // Kitchen <-> Trash room (hallway exists)
    this->kg_->create_edge("connected", "kitchen_counter", "dumpster");
    this->kg_->create_edge("connected", "dumpster", "kitchen_counter");

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

      // Remove last _<name> suffix from location
      size_t last_underscore = location.find_last_of('_');
      if (last_underscore != std::string::npos) {
        location = location.substr(0, last_underscore);
      }

      this->kg_->create_node(robot_name, "robot");
      this->kg_->create_edge("robot_at", robot_name, location);
      this->kg_->create_edge("gripper_free", robot_name, robot_name);
      this->kg_->create_edge("battery_ok", robot_name, robot_name);
    }

    // ── Print initial state ──
    std::cout << "Pyrobosim Restaurant Demo - Knowledge Graph" << std::endl;
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

    std::cout << "\nInitial state loaded. Starting customer simulation ..."
              << std::endl;
  }

  // ── Table thread ──────────────────────────────────────────────────────────
  void table_loop(const std::string &table) {
    std::mt19937 rng(std::random_device{}());

    while (this->running_.load() && rclcpp::ok()) {
      // Random delay before next customer group
      std::uniform_real_distribution<double> delay_dist(this->arrival_min_,
                                                        this->arrival_max_);
      double delay = delay_dist(rng);
      RCLCPP_INFO(this->get_logger(), "[%s] Next group arriving in %.1f s ...",
                  table.c_str(), delay);
      this->sleep_interruptible(delay);
      if (!this->running_.load() || !rclcpp::ok())
        break;

      this->handle_customer_group(table, rng);
    }
  }

  // ── Customer group lifecycle ──────────────────────────────────────────────
  void handle_customer_group(const std::string &table, std::mt19937 &rng) {
    // Record start time for order duration measurement
    auto start_time_steady = std::chrono::steady_clock::now();
    auto start_time_ros = this->now();

    // Determine group size (1–max_group_size_)
    std::uniform_int_distribution<int> group_dist(1, max_group_size_);
    int group_size = group_dist(rng);

    // Set to collect robot names involved in this order
    std::set<std::string> involved_robots;

    RCLCPP_INFO(this->get_logger(), "[%s] Group of %d arrived!", table.c_str(),
                group_size);
    this->set_table_state(table, TableState::ORDERING);

    // 1. Create "order_pending" fact and "order_ready" goal edge
    //    The planner will send a waiter to the table to take the order
    //    (take_order action requires order_pending and produces order_ready)
    this->kg_->create_edge("order_pending", table, table);

    auto goal_edge =
        this->kg_->create_edge("order_ready", table, "kitchen_counter");
    goal_edge.set_property("is_goal", true);
    this->kg_->update_edge(goal_edge);

    RCLCPP_INFO(this->get_logger(), "[%s] Waiting for order to be taken ...",
                table.c_str());
    // The robot that takes the order must be at the table when the goal fires
    this->wait_for_edge_fulfilled_and_collect_robot(
        "order_ready", table, "kitchen_counter", "robot_at", table,
        involved_robots);

    // 2. Generate orders for all customers in the group
    int order_id;
    {
      std::lock_guard<std::mutex> lock(this->counter_mutex_);
      order_id = ++this->order_counter_;
    }

    std::vector<OrderItem> order_items =
        this->generate_order(table, order_id, group_size, rng);

    // 3. Simulate order preparation time
    std::uniform_real_distribution<double> prep_dist(this->order_prep_min_,
                                                     this->order_prep_max_);
    double prep_time = prep_dist(rng);
    RCLCPP_INFO(this->get_logger(),
                "[%s] Order #%d being prepared (%.1f s) ...", table.c_str(),
                order_id, prep_time);
    this->sleep_interruptible(prep_time);
    if (!this->running_.load() || !rclcpp::ok())
      return;

    this->set_table_state(table, TableState::SERVING);

    // 4. Spawn items in pyrobosim and set KG delivery goals
    std::vector<OrderItem> spawned_items =
        this->spawn_and_set_goals(table, order_items);

    // Collect item names to track via "carrying" edges during delivery
    std::vector<std::string> delivery_item_names;
    for (const auto &item : spawned_items) {
      delivery_item_names.push_back(item.name);
    }

    // 5. Wait for all items to be delivered; track robots via "carrying" edges
    RCLCPP_INFO(this->get_logger(),
                "[%s] Waiting for %zu items to be delivered ...", table.c_str(),
                spawned_items.size());
    this->wait_for_delivery(table, spawned_items, delivery_item_names,
                            involved_robots);
    RCLCPP_INFO(this->get_logger(), "[%s] All items delivered!", table.c_str());

    this->set_table_state(table, TableState::EATING);

    // 6. Customer group eats
    std::uniform_real_distribution<double> eat_dist(this->eating_min_,
                                                    this->eating_max_);
    double eating_time = eat_dist(rng);
    RCLCPP_INFO(this->get_logger(), "[%s] Group eating for %.1f s ...",
                table.c_str(), eating_time);
    this->sleep_interruptible(eating_time);
    if (!this->running_.load() || !rclcpp::ok())
      return;

    this->set_table_state(table, TableState::CLEANING);

    // 7. Replace dishes with waste; remove drinks; set cleanup goals
    std::vector<std::string> waste_items =
        this->replace_with_waste(table, spawned_items, order_items, order_id);

    // 8. Wait for waste to be taken to dumpster; track robots via "carrying"
    RCLCPP_INFO(this->get_logger(),
                "[%s] Waiting for %zu waste items to be cleaned ...",
                table.c_str(), waste_items.size());
    this->wait_for_cleanup(waste_items, involved_robots);
    RCLCPP_INFO(this->get_logger(), "[%s] Table cleaned!", table.c_str());

    // 9. Clean up KG nodes for waste items (leave pyrobosim objects for
    //    dumpster-emptying thread)
    this->cleanup_kg(waste_items);

    this->set_table_state(table, TableState::FREE);
    RCLCPP_INFO(this->get_logger(), "[%s] Table is free again.", table.c_str());

    // 10. Publish order completion report
    auto end_time_ros = this->now();
    auto end_time_steady = std::chrono::steady_clock::now();
    double total_duration_sec =
        std::chrono::duration<double>(end_time_steady - start_time_steady)
            .count();

    omni_plan_pyrobosim_msgs::msg::OrderCompleted msg;
    msg.order_id = order_id;
    msg.table = table;
    msg.group_size = group_size;
    for (const auto &item : order_items) {
      msg.item_categories.push_back(item.category);
    }
    msg.total_duration_sec = total_duration_sec;
    msg.start_time = start_time_ros;
    msg.end_time = end_time_ros;
    for (const auto &robot : involved_robots) {
      msg.robots_involved.push_back(robot);
    }

    this->order_completed_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "[%s] Order #%d completed in %.1f s. Robots: [%s]",
                table.c_str(), order_id, total_duration_sec,
                join(std::vector<std::string>(involved_robots.begin(),
                                              involved_robots.end()),
                     ", ")
                    .c_str());
  }

  // ── Order generation ──────────────────────────────────────────────────────
  std::vector<OrderItem> generate_order(const std::string &table, int order_id,
                                        int group_size,
                                        std::mt19937 & /* rng */) {
    std::vector<OrderItem> items;

    // One dish per table
    items.push_back({"order" + std::to_string(order_id) + "_dish_order",
                     "dish_order", "kitchen_counter"});
    items.push_back({"order" + std::to_string(order_id) + "_drink_order",
                     "drink_order", "bar_counter0"});

    std::vector<std::string> cats;
    for (const auto &item : items)
      cats.push_back(item.category);

    RCLCPP_INFO(this->get_logger(), "[%s] Order #%d (%d customers): [%s]",
                table.c_str(), order_id, group_size, join(cats, ", ").c_str());
    return items;
  }

  // ── Spawn items & set KG goals ────────────────────────────────────────────
  std::vector<OrderItem>
  spawn_and_set_goals(const std::string &table,
                      const std::vector<OrderItem> &order_items) {
    std::vector<OrderItem> spawned;
    for (const auto &item : order_items) {
      // Spawn in pyrobosim
      auto req = std::make_shared<pyrobosim_msgs::srv::AddObject::Request>();
      req->category = item.category;
      req->name = item.name;
      req->parent = item.spawn_location;

      auto resp = this->call_service(this->add_obj_cli_, req);
      if (resp && resp->success) {
        RCLCPP_INFO(this->get_logger(), "  Spawned %s (%s) at %s",
                    item.name.c_str(), item.category.c_str(),
                    item.spawn_location.c_str());
      } else {
        std::string msg = resp ? resp->message : "timeout";
        RCLCPP_ERROR(this->get_logger(), "  Failed to spawn %s: %s",
                     item.name.c_str(), msg.c_str());
        continue;
      }

      // Create KG node for item
      this->kg_->create_node(item.name, "item");

      // Set current location in KG
      this->kg_->create_edge("item_at", item.name, item.spawn_location);

      // Set delivery goal
      auto goal_edge = this->kg_->create_edge("item_at", item.name, table);
      goal_edge.set_property("is_goal", true);
      this->kg_->update_edge(goal_edge);

      spawned.push_back(item);
    }
    return spawned;
  }

  // ── Wait for delivery ─────────────────────────────────────────────────────
  void wait_for_delivery(const std::string &table,
                         const std::vector<OrderItem> &order_items,
                         const std::vector<std::string> &item_names,
                         std::set<std::string> &involved_robots) {
    std::vector<std::string> pending;
    for (const auto &item : order_items)
      pending.push_back(item.name);

    while (!pending.empty() && this->running_.load() && rclcpp::ok()) {
      // A robot is involved if it currently carries one of this order's items
      this->collect_robots_carrying(item_names, involved_robots);

      pending.erase(
          std::remove_if(pending.begin(), pending.end(),
                         [&](const std::string &name) {
                           if (this->kg_->has_edge("item_at", name, table)) {
                             auto edge =
                                 this->kg_->get_edge("item_at", name, table);
                             return !edge.has_property("is_goal") ||
                                    !edge.get_property<bool>("is_goal");
                           }
                           return false;
                         }),
          pending.end());

      if (!pending.empty())
        this->sleep_interruptible(this->poll_interval_);
    }
  }

  // ── Wait for edge fulfilled (non-goal edge exists) ────────────────────────
  /// Waits until the KG edge (type, source, target) exists without is_goal.
  /// Simultaneously records whichever robot has `robot_edge_type(robot,
  /// location_to_watch)` at the moment the goal is fulfilled — that is the
  /// robot that executed the action (e.g. take_order: robot_at(robot, table)).
  void wait_for_edge_fulfilled_and_collect_robot(
      const std::string &type, const std::string &source,
      const std::string &target, const std::string &robot_edge_type,
      const std::string &location_to_watch,
      std::set<std::string> &involved_robots) {
    while (this->running_.load() && rclcpp::ok()) {
      if (this->kg_->has_edge(type, source, target)) {
        auto edge = this->kg_->get_edge(type, source, target);
        if (!edge.has_property("is_goal") ||
            !edge.get_property<bool>("is_goal")) {
          auto nodes = this->kg_->get_nodes();
          for (const auto &node : nodes) {
            if (node.get_type() != "robot")
              continue;
            if (this->kg_->has_edge(robot_edge_type, node.get_name(),
                                    location_to_watch)) {
              involved_robots.insert(node.get_name());
            }
          }
          return;
        }
      }
      this->sleep_interruptible(this->poll_interval_);
    }
  }

  // ── Replace items with waste ──────────────────────────────────────────────
  std::vector<std::string>
  replace_with_waste(const std::string &table,
                     const std::vector<OrderItem> &spawned_items,
                     const std::vector<OrderItem> &all_items, int order_id) {
    // Remove all successfully tracked items from pyrobosim and KG.
    // Retry up to 3 times so that drinks/dishes physically at the table
    // are reliably removed before waste is spawned.
    for (const auto &item : spawned_items) {
      bool removed = false;
      this->sleep_interruptible(2.0);

      auto rm_req =
          std::make_shared<pyrobosim_msgs::srv::RemoveObject::Request>();
      rm_req->name = item.name;
      auto rm_resp = this->call_service(this->rm_obj_cli_, rm_req);

      if (rm_resp && rm_resp->success) {
        RCLCPP_INFO(this->get_logger(), "  Removed %s from world",
                    item.name.c_str());
        removed = true;
      } else if (!removed) {
        RCLCPP_ERROR(this->get_logger(), "  Failed to remove %s; skipping.",
                     item.name.c_str());
      }

      if (this->kg_->has_edge("item_at", item.name, table)) {
        this->kg_->remove_edge(
            this->kg_->get_edge("item_at", item.name, table));
      }
      if (this->kg_->has_node(item.name)) {
        this->kg_->remove_node(this->kg_->get_node(item.name));
      }
    }

    for (const auto &item : all_items) {
      bool was_spawned =
          std::any_of(spawned_items.begin(), spawned_items.end(),
                      [&](const OrderItem &s) { return s.name == item.name; });
      if (was_spawned) {
        continue;
      }
      auto rm_req =
          std::make_shared<pyrobosim_msgs::srv::RemoveObject::Request>();
      rm_req->name = item.name;
      auto rm_resp = this->call_service(this->rm_obj_cli_, rm_req);
      if (rm_resp && rm_resp->success) {
        RCLCPP_WARN(this->get_logger(),
                    "  Removed orphaned %s (spawn was not confirmed)",
                    item.name.c_str());
      }
    }

    // Spawn a single waste item for the whole table
    std::string waste_name = "waste_order" + std::to_string(order_id);

    auto add_req = std::make_shared<pyrobosim_msgs::srv::AddObject::Request>();
    add_req->category = "waste";
    add_req->name = waste_name;
    add_req->parent = table;

    auto add_resp = this->call_service(this->add_obj_cli_, add_req);

    while (!add_resp || !add_resp->success) {
      std::string msg = add_resp ? add_resp->message : "timeout";
      RCLCPP_ERROR(this->get_logger(), "  Failed to spawn %s: %s. Retrying ...",
                   waste_name.c_str(), msg.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
      add_resp = this->call_service(this->add_obj_cli_, add_req);
    }
    RCLCPP_INFO(this->get_logger(), "  Spawned %s at %s", waste_name.c_str(),
                table.c_str());

    // KG node and edges for waste
    this->kg_->create_node(waste_name, "item");
    this->kg_->create_edge("item_at", waste_name, table);

    // Goal: waste should end up at the dumpster
    auto goal_edge = this->kg_->create_edge("item_at", waste_name, "dumpster");
    goal_edge.set_property("is_goal", true);
    this->kg_->update_edge(goal_edge);

    return {waste_name};
  }

  // ── Wait for cleanup ──────────────────────────────────────────────────────
  void wait_for_cleanup(const std::vector<std::string> &waste_items,
                        std::set<std::string> &involved_robots) {
    std::vector<std::string> pending = waste_items;

    while (!pending.empty() && this->running_.load() && rclcpp::ok()) {
      // A robot is involved if it currently carries one of the waste items
      this->collect_robots_carrying(waste_items, involved_robots);

      pending.erase(
          std::remove_if(
              pending.begin(), pending.end(),
              [&](const std::string &name) {
                if (this->kg_->has_edge("item_at", name, "dumpster")) {
                  auto edge = this->kg_->get_edge("item_at", name, "dumpster");
                  return !edge.has_property("is_goal") ||
                         !edge.get_property<bool>("is_goal");
                }
                return false;
              }),
          pending.end());

      if (!pending.empty())
        this->sleep_interruptible(this->poll_interval_);
    }
  }

  // ── KG cleanup ────────────────────────────────────────────────────────────
  void cleanup_kg(const std::vector<std::string> &waste_items) {
    for (const auto &name : waste_items) {
      if (this->kg_->has_edge("item_at", name, "dumpster")) {
        this->kg_->remove_edge(
            this->kg_->get_edge("item_at", name, "dumpster"));
      }
      if (this->kg_->has_node(name)) {
        this->kg_->remove_node(this->kg_->get_node(name));
      }
    }
  }

  // ── Dumpster-emptying loop ────────────────────────────────────────────────
  void dumpster_loop() {
    while (this->running_.load() && rclcpp::ok()) {
      this->sleep_interruptible(this->dumpster_empty_interval_);
      if (!this->running_.load() || !rclcpp::ok())
        break;

      // Query all objects at the dumpster
      auto req = std::make_shared<pyrobosim_msgs::srv::GetObjects::Request>();
      req->category = "waste";
      auto resp = this->call_service(this->get_objs_cli_, req);
      if (!resp)
        continue;

      int removed = 0;
      for (const auto &obj : resp->objects) {
        // Only remove objects whose parent contains "dumpster"
        if (obj.parent.find("dumpster") == std::string::npos)
          continue;

        auto rm_req =
            std::make_shared<pyrobosim_msgs::srv::RemoveObject::Request>();
        rm_req->name = obj.name;
        auto rm_resp = this->call_service(this->rm_obj_cli_, rm_req);
        if (rm_resp && rm_resp->success) {
          ++removed;
        }
      }
      if (removed > 0) {
        RCLCPP_INFO(this->get_logger(), "[dumpster] Emptied %d waste objects.",
                    removed);
      }
    }
  }

  // ── Robot involvement tracking ────────────────────────────────────────────
  /// Inserts into `involved_robots` every robot that currently has a
  /// `carrying(robot, item)` edge for any of the given item names.
  /// This is the ground-truth signal for pick/place involvement: the planner
  /// sets `carrying(robot, item)` at pick-start and removes it at place-end.
  void collect_robots_carrying(const std::vector<std::string> &item_names,
                               std::set<std::string> &involved_robots) {
    auto nodes = this->kg_->get_nodes();
    for (const auto &node : nodes) {
      if (node.get_type() != "robot")
        continue;
      for (const auto &item : item_names) {
        if (this->kg_->has_edge("carrying", node.get_name(), item)) {
          involved_robots.insert(node.get_name());
        }
      }
    }
  }

  // ── Service call helper ───────────────────────────────────────────────────
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr
  call_service(std::shared_ptr<rclcpp::Client<ServiceT>> client,
               std::shared_ptr<typename ServiceT::Request> request,
               double timeout_sec = 10.0) {
    while (!this->spin_ready_.load() && this->running_.load() && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!this->spin_ready_.load()) {
      return nullptr;
    }
    std::lock_guard<std::mutex> lock(this->service_mutex_);
    if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available after %.1f s",
                   client->get_service_name(), timeout_sec);
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(timeout_sec)) ==
        std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  // ── Helpers ───────────────────────────────────────────────────────────────
  void set_table_state(const std::string &table, TableState state) {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->table_states_[table] = state;
  }

  void sleep_interruptible(double seconds) {
    auto end = std::chrono::steady_clock::now() +
               std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end && this->running_.load() &&
           rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  static std::string join(const std::vector<std::string> &v,
                          const std::string &sep) {
    std::string out;
    for (size_t i = 0; i < v.size(); ++i) {
      if (i > 0)
        out += sep;
      out += v[i];
    }
    return out;
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;

  rclcpp::Publisher<omni_plan_pyrobosim_msgs::msg::OrderCompleted>::SharedPtr
      order_completed_pub_;

  rclcpp::Client<pyrobosim_msgs::srv::RequestWorldState>::SharedPtr
      req_world_state_cli_;
  rclcpp::Client<pyrobosim_msgs::srv::AddObject>::SharedPtr add_obj_cli_;
  rclcpp::Client<pyrobosim_msgs::srv::RemoveObject>::SharedPtr rm_obj_cli_;
  rclcpp::Client<pyrobosim_msgs::srv::GetObjects>::SharedPtr get_objs_cli_;

  double arrival_min_;
  double arrival_max_;
  double eating_min_;
  double eating_max_;
  double order_prep_min_;
  double order_prep_max_;
  double poll_interval_;
  double dumpster_empty_interval_;
  int max_group_size_;

  std::map<std::string, TableState> table_states_;
  std::mutex state_mutex_;

  int order_counter_ = 0;
  std::mutex counter_mutex_;

  std::vector<std::thread> table_threads_;
  std::thread dumpster_thread_;
  std::thread spin_thread_;
  rclcpp::Executor::SharedPtr executor_;
  std::atomic<bool> running_{true};
  std::atomic<bool> spin_ready_{false};
  std::mutex service_mutex_;
};

// ── Entry point ─────────────────────────────────────────────────────────────
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RestaurantCustomerSimConnected>();
  node->join_spin();
  rclcpp::shutdown();
  return 0;
}
