# OmniPlan Pyrobosim

ROS 2 integration of [OmniPlan](https://github.com/mgonzs13/omni_plan) with [pyrobosim](https://github.com/mgonzs13/pyrobosim_ros) for task planning and execution in restaurant and warehouse domains. This repository contains the simulator integration and demo worlds only.

## Packages

| Package                       | Description                                                                                                                   |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `omni_plan_pyrobosim`         | Action nodes bridging OmniPlan to pyrobosim: navigation, pick/place, charge, customer/package simulation, battery monitoring. |
| `omni_plan_pyrobosim_bringup` | Launch files, planner parameters and pyrobosim worlds (1/3/5/10 robots).                                                      |
| `omni_plan_pyrobosim_msgs`    | Messages published by the simulation (`OrderCompleted`, `PackageDelivered`).                                                  |

## Installation

```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/mgonzs13/omni_plan
git clone https://github.com/mgonzs13/omni_plan_pyrobosim
git clone https://github.com/mgonzs13/pyrobosim_ros
vcs import src < src/omni_plan/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Demos

```shell
# Restaurant, 3 robots, knowledge graph backend
ros2 launch omni_plan_pyrobosim_bringup restaurant.launch.py robot_number:=3

# Restaurant, 5 robots, knowledge base backend
ros2 launch omni_plan_pyrobosim_bringup restaurant.launch.py \
  robot_number:=5 use_knowledge_base:=true

# Warehouse, 3 robots
ros2 launch omni_plan_pyrobosim_bringup warehouse.launch.py robot_number:=3
```

`robot_number` accepts `1`, `3`, `5`, `10` for the restaurant and `3`, `5`, `10` for the warehouse. Other useful arguments are `use_connected_sim:=true` and `state_machine_file:=<name>.xml`.

## Docker

The `Dockerfile` builds on the `omniplan:latest` base image and includes the pyrobosim dependencies and worlds:

```shell
docker build -t omni-plan-pyrobosim .
```

## Paper artifacts

The experiment code and aggregated results for each paper live in separate repositories:

| Paper | Repository |
|---|---|
| Full OmniPlan stack: planner × MRTA × VAL × dispatcher | _TBD_ |
| Planner × reactive replanning strategy (MRTA) | _TBD_ |
| Planner × knowledge representation | _TBD_ |
| Navigation variants × planner × MRTA | _TBD_ |

Each repository pins the commit it was produced with and keeps its aggregated results,
figures and LaTeX tables under `analysis_output/`.

## License

Apache-2.0, see [`LICENSE`](LICENSE).
