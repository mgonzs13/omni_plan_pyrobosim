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

/// @file restaurant_customer_sim_kb.cpp
/// @brief Restaurant Customer Simulator (KnowledgeBase backend)
///
/// KnowledgeBase version of restaurant_customer_sim.cpp.
/// Writes state to KnowledgeBase via KnowledgeBaseClient instead of
/// the in-process KnowledgeGraph singleton.

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

#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"

#include "omni_plan_knowledge_base/knowledge_base_client.hpp"
#include "omni_plan_pyrobosim_msgs/msg/order_completed.hpp"
#include "pyrobosim_msgs/srv/add_object.hpp"
#include "pyrobosim_msgs/srv/get_objects.hpp"
#include "pyrobosim_msgs/srv/remove_object.hpp"
#include "pyrobosim_msgs/srv/request_world_state.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

static const std::vector<std::string> TABLES = {"table1", "table2", "table3",
                                                "table4", "table5", "table6"};

enum class TableState { FREE, ORDERING, SERVING, EATING, CLEANING };

struct OrderItem {
  std::string name;
  std::string category;
  std::string spawn_location;
};

class RestaurantCustomerSimKb : public rclcpp::Node {
public:
  RestaurantCustomerSimKb() : Node("restaurant_customer_sim_kb") {

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

    this->executor_ =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->spin_thread_ = std::thread([this]() {
      this->executor_->add_node(this->get_node_base_interface());
      this->executor_->spin();
    });

    this->req_world_state_cli_ =
        this->create_client<pyrobosim_msgs::srv::RequestWorldState>(
            "request_world_state");
    this->add_obj_cli_ =
        this->create_client<pyrobosim_msgs::srv::AddObject>("add_object");
    this->rm_obj_cli_ =
        this->create_client<pyrobosim_msgs::srv::RemoveObject>("remove_object");
    this->get_objs_cli_ =
        this->create_client<pyrobosim_msgs::srv::GetObjects>("get_objects");

    // KnowledgeBase client
    this->kb_ = std::make_shared<omni_plan_knowledge_base::KnowledgeBaseClient>(
        "restaurant_customer_sim_kb_client");
    this->kb_->clear();
    this->initialize_knowledge_base();

    this->order_completed_pub_ =
        this->create_publisher<omni_plan_pyrobosim_msgs::msg::OrderCompleted>(
            "order_completed", 10);

    RCLCPP_INFO(this->get_logger(), "Waiting for pyrobosim services ...");
    this->req_world_state_cli_->wait_for_service(std::chrono::seconds(60));
    this->add_obj_cli_->wait_for_service(std::chrono::seconds(60));
    this->rm_obj_cli_->wait_for_service(std::chrono::seconds(60));
    this->get_objs_cli_->wait_for_service(std::chrono::seconds(60));
    RCLCPP_INFO(this->get_logger(), "All pyrobosim services available.");

    for (const auto &t : TABLES) {
      this->table_states_[t] = TableState::FREE;
    }

    for (const auto &t : TABLES) {
      this->table_threads_.emplace_back(
          std::thread(&RestaurantCustomerSimKb::table_loop, this, t));
    }

    this->dumpster_thread_ =
        std::thread(&RestaurantCustomerSimKb::dumpster_loop, this);

    RCLCPP_INFO(this->get_logger(), "Customer simulation (KB) started.");
  }

  ~RestaurantCustomerSimKb() override {
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
  // ── KB helpers ─────────────────────────────────────────────────────────
  bool has_fact(const std::string &name,
                const std::vector<std::string> &args) const {
    auto facts = this->kb_->get_facts(name);
    for (const auto &f : facts) {
      if (f.get_args() == args)
        return true;
    }
    return false;
  }

  bool has_goal(const std::string &name,
                const std::vector<std::string> &args) const {
    auto goals = this->kb_->get_goals();
    for (const auto &g : goals) {
      if (g.get_name() == name && g.get_args() == args)
        return true;
    }
    return false;
  }

  bool has_object(const std::string &name) const {
    auto objs = this->kb_->get_objects();
    for (const auto &o : objs) {
      if (o.get_name() == name)
        return true;
    }
    return false;
  }

  std::vector<omni_plan::pddl::Object>
  get_objects_by_type(const std::string &type) const {
    std::vector<omni_plan::pddl::Object> result;
    auto objs = this->kb_->get_objects();
    for (const auto &o : objs) {
      if (o.get_type() == type)
        result.push_back(o);
    }
    return result;
  }

  // ── KB world initialization ───────────────────────────────────────────
  void initialize_knowledge_base() {
    // Types
    this->kb_->add_type("restaurant");
    this->kb_->add_type("room");
    this->kb_->add_type("location");
    this->kb_->add_type("item");
    this->kb_->add_type("robot");

    // Predicate definitions (type args matching PDDL)
    // Self-referential KG edges → unary PDDL predicates
    this->kb_->add_predicate("battery_ok", {"robot"});
    this->kb_->add_predicate("gripper_free", {"robot"});
    this->kb_->add_predicate("order_pending", {"location"});
    this->kb_->add_predicate("is_charger", {"location"});
    // Cross KG edges → binary PDDL predicates
    this->kb_->add_predicate("robot_at", {"robot", "location"});
    this->kb_->add_predicate("item_at", {"item", "location"});
    this->kb_->add_predicate("order_ready", {"location", "location"});
    this->kb_->add_predicate("carrying", {"robot", "item"});
    this->kb_->add_predicate("has", {"restaurant", "room"});
    this->kb_->add_predicate("in", {"location", "room"});

    // Objects
    this->kb_->add_object("restaurant", "restaurant");
    this->kb_->add_object("entrance", "room");
    this->kb_->add_object("dining_room", "room");
    this->kb_->add_object("kitchen", "room");
    this->kb_->add_object("bar", "room");
    this->kb_->add_object("trash_room", "room");
    this->kb_->add_object("reception", "location");
    this->kb_->add_object("kitchen_counter", "location");
    this->kb_->add_object("bar_counter0", "location");
    this->kb_->add_object("dumpster", "location");
    this->kb_->add_object("charger1", "location");
    this->kb_->add_object("charger2", "location");
    this->kb_->add_object("charger3", "location");
    this->kb_->add_object("charger4", "location");
    this->kb_->add_object("charger5", "location");

    for (int i = 1; i <= 6; ++i) {
      this->kb_->add_object("table" + std::to_string(i), "location");
    }

    this->kb_->add_object("tray1", "item");

    // Facts (value args matching PDDL convention)
    // Binary facts
    this->kb_->add_fact("has", {"restaurant", "entrance"});
    this->kb_->add_fact("has", {"restaurant", "dining_room"});
    this->kb_->add_fact("has", {"restaurant", "kitchen"});
    this->kb_->add_fact("has", {"restaurant", "bar"});
    this->kb_->add_fact("has", {"restaurant", "trash_room"});
    this->kb_->add_fact("in", {"reception", "entrance"});
    this->kb_->add_fact("in", {"kitchen_counter", "kitchen"});
    this->kb_->add_fact("in", {"dumpster", "trash_room"});
    this->kb_->add_fact("in", {"bar_counter0", "bar"});
    this->kb_->add_fact("in", {"charger1", "kitchen"});
    this->kb_->add_fact("in", {"charger2", "bar"});
    this->kb_->add_fact("in", {"charger3", "trash_room"});
    this->kb_->add_fact("in", {"charger4", "dining_room"});
    this->kb_->add_fact("in", {"charger5", "entrance"});

    for (int i = 1; i <= 6; ++i) {
      this->kb_->add_fact("in", {"table" + std::to_string(i), "dining_room"});
    }

    this->kb_->add_fact("item_at", {"tray1", "reception"});

    // Unary facts (self-referential edges in KG)
    this->kb_->add_fact("is_charger", {"charger1"});
    this->kb_->add_fact("is_charger", {"charger2"});
    this->kb_->add_fact("is_charger", {"charger3"});
    this->kb_->add_fact("is_charger", {"charger4"});
    this->kb_->add_fact("is_charger", {"charger5"});

    // Robot state
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

      size_t last_underscore = location.find_last_of('_');
      if (last_underscore != std::string::npos) {
        location = location.substr(0, last_underscore);
      }

      this->kb_->add_object(robot_name, "robot");
      this->kb_->add_fact("robot_at", {robot_name, location});
      this->kb_->add_fact("gripper_free", {robot_name});
      this->kb_->add_fact("battery_ok", {robot_name});
    }

    // Print initial state
    std::cout << "Pyrobosim Restaurant Demo - Knowledge Base" << std::endl;
    std::cout << "============================================" << std::endl;

    auto objects = this->kb_->get_objects();
    std::cout << "Objects (" << objects.size() << "):" << std::endl;
    for (const auto &obj : objects) {
      std::cout << "  " << obj.get_name() << " (" << obj.get_type() << ")"
                << std::endl;
    }

    auto facts = this->kb_->get_facts();
    std::cout << "Facts (" << facts.size() << "):" << std::endl;
    for (const auto &f : facts) {
      std::cout << "  " << f.to_pddl(true) << std::endl;
    }

    std::cout << "\nInitial state loaded. Starting customer simulation ..."
              << std::endl;
  }

  // ── Table thread ──────────────────────────────────────────────────────
  void table_loop(const std::string &table) {
    std::mt19937 rng(std::random_device{}());

    while (this->running_.load() && rclcpp::ok()) {
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

  // ── Customer group lifecycle ──────────────────────────────────────────
  void handle_customer_group(const std::string &table, std::mt19937 &rng) {
    auto start_time_steady = std::chrono::steady_clock::now();
    auto start_time_ros = this->now();

    std::uniform_int_distribution<int> group_dist(1, max_group_size_);
    int group_size = group_dist(rng);

    std::set<std::string> involved_robots;

    RCLCPP_INFO(this->get_logger(), "[%s] Group of %d arrived!", table.c_str(),
                group_size);
    this->set_table_state(table, TableState::ORDERING);

    // 1. Create order_pending fact and order_ready goal
    this->kb_->add_fact("order_pending", {table});
    this->kb_->remove_fact("order_ready", {table, "kitchen_counter"});
    this->kb_->add_goal("order_ready", {table, "kitchen_counter"});

    RCLCPP_INFO(this->get_logger(), "[%s] Waiting for order to be taken ...",
                table.c_str());
    this->wait_for_goal_fulfilled_and_collect_robot(
        "order_ready", {table, "kitchen_counter"}, "robot_at", table,
        involved_robots);

    // 2. Generate orders
    int order_id;
    {
      std::lock_guard<std::mutex> lock(this->counter_mutex_);
      order_id = ++this->order_counter_;
    }

    std::vector<OrderItem> order_items =
        this->generate_order(table, order_id, group_size, rng);

    // 3. Order preparation time
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

    // 4. Spawn items and set KB delivery goals
    std::vector<OrderItem> spawned_items =
        this->spawn_and_set_goals(table, order_items);

    std::vector<std::string> delivery_item_names;
    for (const auto &item : spawned_items) {
      delivery_item_names.push_back(item.name);
    }

    // 5. Wait for delivery
    RCLCPP_INFO(this->get_logger(),
                "[%s] Waiting for %zu items to be delivered ...", table.c_str(),
                spawned_items.size());
    this->wait_for_delivery(table, spawned_items, delivery_item_names,
                            involved_robots);
    RCLCPP_INFO(this->get_logger(), "[%s] All items delivered!", table.c_str());

    this->set_table_state(table, TableState::EATING);

    // 6. Eating
    std::uniform_real_distribution<double> eat_dist(this->eating_min_,
                                                    this->eating_max_);
    double eating_time = eat_dist(rng);
    RCLCPP_INFO(this->get_logger(), "[%s] Group eating for %.1f s ...",
                table.c_str(), eating_time);
    this->sleep_interruptible(eating_time);
    if (!this->running_.load() || !rclcpp::ok())
      return;

    this->set_table_state(table, TableState::CLEANING);

    // 7. Clean up and create waste
    std::vector<std::string> waste_items =
        this->replace_with_waste(table, spawned_items, order_items, order_id);

    // 8. Wait for cleanup
    RCLCPP_INFO(this->get_logger(),
                "[%s] Waiting for %zu waste items to be cleaned ...",
                table.c_str(), waste_items.size());
    this->wait_for_cleanup(waste_items, involved_robots);
    RCLCPP_INFO(this->get_logger(), "[%s] Table cleaned!", table.c_str());

    // 9. Clean up KB
    this->cleanup_kb(waste_items);

    this->set_table_state(table, TableState::FREE);
    RCLCPP_INFO(this->get_logger(), "[%s] Table is free again.", table.c_str());

    // 10. Publish completion
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

  // ── Order generation ──────────────────────────────────────────────────
  std::vector<OrderItem> generate_order(const std::string &table, int order_id,
                                        int group_size,
                                        std::mt19937 & /* rng */) {
    std::vector<OrderItem> items;
    std::uniform_real_distribution<double> drink_chance(0.0, 1.0);

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

  // ── Spawn items & set KB goals ────────────────────────────────────────
  std::vector<OrderItem>
  spawn_and_set_goals(const std::string &table,
                      const std::vector<OrderItem> &order_items) {
    std::vector<OrderItem> spawned;
    for (const auto &item : order_items) {
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

      this->kb_->add_object(item.name, "item");
      this->kb_->add_fact("item_at", {item.name, item.spawn_location});
      this->kb_->add_goal("item_at", {item.name, table});

      spawned.push_back(item);
    }
    return spawned;
  }

  // ── Wait for delivery ─────────────────────────────────────────────────
  void wait_for_delivery(const std::string &table,
                         const std::vector<OrderItem> &order_items,
                         const std::vector<std::string> &item_names,
                         std::set<std::string> &involved_robots) {
    std::vector<std::string> pending;
    for (const auto &item : order_items)
      pending.push_back(item.name);

    while (!pending.empty() && this->running_.load() && rclcpp::ok()) {
      this->collect_robots_carrying(item_names, involved_robots);

      pending.erase(
          std::remove_if(pending.begin(), pending.end(),
                         [&](const std::string &name) {
                           if (this->has_fact("item_at", {name, table})) {
                             return !this->has_goal("item_at", {name, table});
                           }
                           return false;
                         }),
          pending.end());

      if (!pending.empty())
        this->sleep_interruptible(this->poll_interval_);
    }
  }

  // ── Wait for goal fulfilled (fact exists, goal removed) ───────────────
  void wait_for_goal_fulfilled_and_collect_robot(
      const std::string &type, const std::vector<std::string> &args,
      const std::string &robot_edge_type, const std::string &location_to_watch,
      std::set<std::string> &involved_robots) {
    while (this->running_.load() && rclcpp::ok()) {
      if (this->has_fact(type, args)) {
        if (!this->has_goal(type, args)) {
          auto robots = this->get_objects_by_type("robot");
          for (const auto &robot : robots) {
            if (this->has_fact(robot_edge_type,
                               {robot.get_name(), location_to_watch})) {
              involved_robots.insert(robot.get_name());
            }
          }
          return;
        }
      }
      this->sleep_interruptible(this->poll_interval_);
    }
  }

  // ── Replace items with waste ──────────────────────────────────────────
  std::vector<std::string>
  replace_with_waste(const std::string &table,
                     const std::vector<OrderItem> &spawned_items,
                     const std::vector<OrderItem> &all_items, int order_id) {
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

      if (this->has_fact("item_at", {item.name, table})) {
        this->kb_->remove_fact("item_at", {item.name, table});
      }
      if (this->has_object(item.name)) {
        this->kb_->remove_object(item.name, "item");
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

    this->kb_->add_object(waste_name, "item");
    this->kb_->add_fact("item_at", {waste_name, table});
    this->kb_->add_goal("item_at", {waste_name, "dumpster"});

    return {waste_name};
  }

  // ── Wait for cleanup ──────────────────────────────────────────────────
  void wait_for_cleanup(const std::vector<std::string> &waste_items,
                        std::set<std::string> &involved_robots) {
    std::vector<std::string> pending = waste_items;

    while (!pending.empty() && this->running_.load() && rclcpp::ok()) {
      this->collect_robots_carrying(waste_items, involved_robots);

      pending.erase(
          std::remove_if(pending.begin(), pending.end(),
                         [&](const std::string &name) {
                           if (this->has_fact("item_at", {name, "dumpster"})) {
                             return !this->has_goal("item_at",
                                                    {name, "dumpster"});
                           }
                           return false;
                         }),
          pending.end());

      if (!pending.empty())
        this->sleep_interruptible(this->poll_interval_);
    }
  }

  // ── KB cleanup ────────────────────────────────────────────────────────
  void cleanup_kb(const std::vector<std::string> &waste_items) {
    for (const auto &name : waste_items) {
      if (this->has_fact("item_at", {name, "dumpster"})) {
        this->kb_->remove_fact("item_at", {name, "dumpster"});
      }
      if (this->has_object(name)) {
        this->kb_->remove_object(name, "item");
      }
    }
  }

  // ── Dumpster-emptying loop ────────────────────────────────────────────
  void dumpster_loop() {
    while (this->running_.load() && rclcpp::ok()) {
      this->sleep_interruptible(this->dumpster_empty_interval_);
      if (!this->running_.load() || !rclcpp::ok())
        break;

      auto req = std::make_shared<pyrobosim_msgs::srv::GetObjects::Request>();
      req->category = "waste";
      auto resp = this->call_service(this->get_objs_cli_, req);
      if (!resp)
        continue;

      int removed = 0;
      for (const auto &obj : resp->objects) {
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

  // ── Robot involvement tracking ────────────────────────────────────────
  void collect_robots_carrying(const std::vector<std::string> &item_names,
                               std::set<std::string> &involved_robots) {
    auto robots = this->get_objects_by_type("robot");
    for (const auto &robot : robots) {
      for (const auto &item : item_names) {
        if (this->has_fact("carrying", {robot.get_name(), item})) {
          involved_robots.insert(robot.get_name());
        }
      }
    }
  }

  // ── Service call helper ───────────────────────────────────────────────
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

  // ── Helpers ───────────────────────────────────────────────────────────
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

  // ── Members ───────────────────────────────────────────────────────────
  std::shared_ptr<omni_plan_knowledge_base::KnowledgeBaseClient> kb_;

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
};

// ── Entry point ─────────────────────────────────────────────────────────
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RestaurantCustomerSimKb>();
  node->join_spin();
  rclcpp::shutdown();
  return 0;
}
