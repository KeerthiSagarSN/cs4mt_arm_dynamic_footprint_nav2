##################################################################
# Copyright 2025 IMR Robotics & Automation Group
# Author: Keerthi Sagar
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
##################################################################
#
# CORESENSE Project - RAMP Task
# cs4mt_armfootprint_aware_navigation.launch.py
#
# Launches the arm footprint cognitive module for real-time
# footprint projection onto Nav2 costmaps.
#
# Usage:
#   ros2 launch cs4mt_armfootprint_aware_navigation \
#     cs4mt_armfootprint_aware_navigation.launch.py
#
#   With circular base (e.g. TIAGo):
#   ros2 launch cs4mt_armfootprint_aware_navigation \
#     cs4mt_armfootprint_aware_navigation.launch.py \
#     use_circular_base:=true base_radius:=0.30 config:=params_tiago.yaml
#
#   With rectangular base (e.g. MPO-700):
#   ros2 launch cs4mt_armfootprint_aware_navigation \
#     cs4mt_armfootprint_aware_navigation.launch.py \
#     use_circular_base:=false
##################################################################

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    pkg_dir = get_package_share_directory('cs4mt_armfootprint_aware_navigation')

    # Resolve all launch arguments to strings at launch time
    config_filename   = LaunchConfiguration('config').perform(context)
    use_circular_base = LaunchConfiguration('use_circular_base').perform(context)
    base_radius       = LaunchConfiguration('base_radius').perform(context)

    config_file = os.path.join(pkg_dir, 'config', config_filename)

    if not os.path.exists(config_file):
        raise FileNotFoundError(
            f'[ArmFootprint] Config file not found: {config_file}')

    arm_footprint_module_cmd = Node(
        package='cs4mt_armfootprint_aware_navigation',
        executable='arm_footprint_cognitive_module',
        name='arm_footprint_cognitive_module',
        output='screen',
        parameters=[
            config_file,
            {
                'use_circular_base': use_circular_base == 'true',
                'base_radius':       float(base_radius),
            }
        ],
    )

    return [arm_footprint_module_cmd]


def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )

    declare_config_cmd = DeclareLaunchArgument(
        'config',
        default_value='params.yaml',
        description='Config file to load from the config directory. '
                    'Use params.yaml for MPO-700, params_tiago.yaml for TIAGo.'
    )

    declare_use_circular_base_cmd = DeclareLaunchArgument(
        'use_circular_base',
        default_value='false',
        description='Use circular base geometry (true) or rectangular base (false). '
                    'Set true for circular robots such as TIAGo. '
                    'Set false for rectangular bases such as MPO-700.'
    )

    declare_base_radius_cmd = DeclareLaunchArgument(
        'base_radius',
        default_value='0.60',
        description='Radius of the circular base in metres. '
                    'Only used when use_circular_base is true. '
                    'For MPO-700 circumscribed radius: 0.60m. '
                    'For TIAGo: 0.30m.'
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_config_cmd)
    ld.add_action(declare_use_circular_base_cmd)
    ld.add_action(declare_base_radius_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
