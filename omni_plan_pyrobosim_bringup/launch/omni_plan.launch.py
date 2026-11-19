# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("omni_plan_pyrobosim_bringup"),
            "params",
            "pyrobosim_restaurant_demo.yaml",
        ),
        description="Config file to use",
    )

    ld = LaunchDescription()
    ld.add_action(config_file_cmd)

    ld.add_action(
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
                "run_knowledge_graph_viewer": "True",
            }.items(),
        )
    )
    return ld
