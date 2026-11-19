cmake_minimum_required(VERSION 3.5)
project(omni_plan_pyrobosim_bringup)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY params launch pyrobosim_worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
