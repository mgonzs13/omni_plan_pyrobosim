cmake_minimum_required(VERSION 3.5)
project(omni_plan_pyrobosim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(omni_plan_msgs REQUIRED)
find_package(omni_plan_knowledge_base REQUIRED)
find_package(knowledge_graph REQUIRED)
find_package(pyrobosim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(poirot REQUIRED)
find_package(omni_plan_pyrobosim_msgs REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(ACTION_DEPENDENCIES
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  omni_plan::omni_plan
  knowledge_graph::knowledge_graph
  ${pyrobosim_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  yasmin_ros::yasmin_ros
  poirot::poirot
)

# ============================================================================
# Demo: Pyrobosim Restaurant
# ============================================================================
# Actions
add_library(navigate_local_action SHARED src/omni_plan_pyrobosim/navigate_local_action.cpp)
target_link_libraries(navigate_local_action PUBLIC ${ACTION_DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS navigate_local_action LIBRARY DESTINATION lib)

add_library(navigate_global_action SHARED src/omni_plan_pyrobosim/navigate_global_action.cpp)
target_link_libraries(navigate_global_action PUBLIC ${ACTION_DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS navigate_global_action LIBRARY DESTINATION lib)

add_library(navigate_to_charger_action SHARED src/omni_plan_pyrobosim/navigate_to_charger_action.cpp)
target_link_libraries(navigate_to_charger_action PUBLIC ${ACTION_DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS navigate_to_charger_action LIBRARY DESTINATION lib)

add_library(pick_action SHARED src/omni_plan_pyrobosim/pick_action.cpp)
target_link_libraries(pick_action PUBLIC ${ACTION_DEPENDENCIES})
install(TARGETS pick_action LIBRARY DESTINATION lib)

add_library(place_action SHARED src/omni_plan_pyrobosim/place_action.cpp)
target_link_libraries(place_action PUBLIC ${ACTION_DEPENDENCIES})
install(TARGETS place_action LIBRARY DESTINATION lib)

add_library(take_order_action SHARED src/omni_plan_pyrobosim/take_order_action.cpp)
target_link_libraries(take_order_action PUBLIC ${ACTION_DEPENDENCIES})
install(TARGETS take_order_action LIBRARY DESTINATION lib)

add_library(charge_action SHARED src/omni_plan_pyrobosim/charge_action.cpp)
target_link_libraries(charge_action PUBLIC ${ACTION_DEPENDENCIES})
install(TARGETS charge_action LIBRARY DESTINATION lib)

add_library(navigate_bare_action SHARED src/omni_plan_pyrobosim/navigate_bare_action.cpp)
target_link_libraries(navigate_bare_action PUBLIC ${ACTION_DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS navigate_bare_action LIBRARY DESTINATION lib)

add_library(navigate_connected_action SHARED src/omni_plan_pyrobosim/navigate_connected_action.cpp)
target_link_libraries(navigate_connected_action PUBLIC ${ACTION_DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS navigate_connected_action LIBRARY DESTINATION lib)

# Battery monitor executable
add_executable(battery_monitor src/omni_plan_pyrobosim/battery_monitor.cpp)
target_link_libraries(battery_monitor
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${pyrobosim_msgs_TARGETS}
)
install(TARGETS battery_monitor DESTINATION lib/${PROJECT_NAME})

# Customer simulator executable (C++)
add_executable(restaurant_customer_sim src/omni_plan_pyrobosim/restaurant_customer_sim.cpp)
target_link_libraries(restaurant_customer_sim
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${knowledge_graph_msgs_TARGETS}
  ${pyrobosim_msgs_TARGETS}
  ${omni_plan_pyrobosim_msgs_TARGETS}
)
install(TARGETS restaurant_customer_sim DESTINATION lib/${PROJECT_NAME})

# Customer simulator executable (C++)
add_executable(warehouse_package_sim src/omni_plan_pyrobosim/warehouse_package_sim.cpp)
target_link_libraries(warehouse_package_sim
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${knowledge_graph_msgs_TARGETS}
  ${pyrobosim_msgs_TARGETS}
  ${omni_plan_pyrobosim_msgs_TARGETS}
)
install(TARGETS warehouse_package_sim DESTINATION lib/${PROJECT_NAME})

add_executable(restaurant_customer_sim_connected src/omni_plan_pyrobosim/restaurant_customer_sim_connected.cpp)
target_link_libraries(restaurant_customer_sim_connected
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${knowledge_graph_msgs_TARGETS}
  ${pyrobosim_msgs_TARGETS}
  ${omni_plan_pyrobosim_msgs_TARGETS}
)
install(TARGETS restaurant_customer_sim_connected DESTINATION lib/${PROJECT_NAME})

add_executable(warehouse_package_sim_connected src/omni_plan_pyrobosim/warehouse_package_sim_connected.cpp)
target_link_libraries(warehouse_package_sim_connected
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${knowledge_graph_msgs_TARGETS}
  ${pyrobosim_msgs_TARGETS}
  ${omni_plan_pyrobosim_msgs_TARGETS}
)
install(TARGETS warehouse_package_sim_connected DESTINATION lib/${PROJECT_NAME})

# KB customer simulator executable
add_executable(restaurant_customer_sim_kb src/omni_plan_pyrobosim/restaurant_customer_sim_kb.cpp)
target_link_libraries(restaurant_customer_sim_kb
  rclcpp::rclcpp
  omni_plan_knowledge_base::knowledge_base_client
  omni_plan::omni_plan
  ${pyrobosim_msgs_TARGETS}
  ${omni_plan_pyrobosim_msgs_TARGETS}
)
install(TARGETS restaurant_customer_sim_kb DESTINATION lib/${PROJECT_NAME})

# KB battery monitor executable
add_executable(battery_monitor_kb src/omni_plan_pyrobosim/battery_monitor_kb.cpp)
target_link_libraries(battery_monitor_kb
  rclcpp::rclcpp
  omni_plan_knowledge_base::knowledge_base_client
  ${pyrobosim_msgs_TARGETS}
)
install(TARGETS battery_monitor_kb DESTINATION lib/${PROJECT_NAME})

# Install python scripts executables
install(PROGRAMS
  scripts/subscribe_order_completed.py
  scripts/subscribe_package_delivered.py
  scripts/subscribe_plan_execution.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
