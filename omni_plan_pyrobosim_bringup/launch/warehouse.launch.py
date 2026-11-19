# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():

    def get_world_file(context):
        robot_number = int(LaunchConfiguration("robot_number").perform(context))
        world_file = os.path.join(
            get_package_share_directory("omni_plan_pyrobosim_bringup"),
            "pyrobosim_worlds",
            f"warehouse_world_{robot_number}.yaml",
        )
        return [
            DeclareLaunchArgument(
                "world_file",
                default_value=world_file,
                description="World file to use",
            )
        ]

    def run_omni_plan(context):
        state_machine_file = LaunchConfiguration("state_machine_file").perform(context)

        # Check if the file is a global path or a XML file in the omni_plan package
        if not os.path.isfile(state_machine_file):
            if not state_machine_file.endswith(".xml"):
                raise ValueError(
                    f"State machine file '{state_machine_file}' is not a valid file or XML file in the omni_plan package"
                )

            state_machine_file = os.path.join(
                get_package_share_directory("omni_plan"),
                "state_machines",
                state_machine_file,
            )

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("omni_plan_bringup"),
                        "launch",
                        "omni_plan.launch.py",
                    ),
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("config_file"),
                    "state_machine_file": state_machine_file,
                    "run_knowledge_graph_viewer": "True",
                }.items(),
            )
        ]

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "robot_number",
            default_value="3",
            description="Number of robots (choices: 3, 5, 10)",
            choices=["3", "5", "10"],
        )
    )
    ld.add_action(OpaqueFunction(function=get_world_file))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("omni_plan_pyrobosim_bringup"),
                    "launch",
                    "pyrobosim_world.launch.py",
                ),
            ),
            launch_arguments={
                "world_file": LaunchConfiguration("world_file"),
            }.items(),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                get_package_share_directory("omni_plan_pyrobosim_bringup"),
                "params",
                "omni_plan_warehouse.yaml",
            ),
            description="Config file to use",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "state_machine_file",
            default_value="planning_sm.xml",
            description="State machine file to use",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_connected_sim",
            default_value="false",
            description="Use connected sim node (publishes connected facts)",
        )
    )
    ld.add_action(OpaqueFunction(function=run_omni_plan))
    ld.add_action(
        Node(
            package="omni_plan_pyrobosim",
            executable="subscribe_package_delivered.py",
            name="subscribe_package_delivered",
            output="both",
        )
    )
    ld.add_action(
        Node(
            package="omni_plan_pyrobosim",
            executable="subscribe_plan_execution.py",
            name="subscribe_plan_execution",
            output="both",
        )
    )
    ld.add_action(
        Node(
            package="poirot_recorder",
            executable="poirot_recorder_node",
            name="poirot_recorder_node",
            output="both",
        )
    )
    ld.add_action(
        Node(
            package="omni_plan_pyrobosim",
            executable="battery_monitor",
            name="battery_monitor",
            output="both",
        )
    )
    def launch_sim_node(context):
        use_connected = LaunchConfiguration("use_connected_sim").perform(context) == "true"
        executable = "warehouse_package_sim_connected" if use_connected else "warehouse_package_sim"
        name = "warehouse_package_sim_connected" if use_connected else "warehouse_package_sim"
        return [
            Node(
                package="omni_plan_pyrobosim",
                executable=executable,
                name=name,
                output="both",
            )
        ]

    ld.add_action(OpaqueFunction(function=launch_sim_node))
    return ld
