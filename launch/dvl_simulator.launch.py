# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
# Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
# All rights reserved.

import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare the path to the config YAML file
    config_file_path = os.path.join(
        get_package_share_directory("dvl_simulator_package"),  # noqa
        "config",  # noqa
        "waterlinked_a50.yaml",  # noqa
    )

    # Open the YAML file and load the parameters
    with open(config_file_path, "r") as file:
        config = yaml.safe_load(file)

    topic_name_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="/auv/odometry",
        description="Topic name of the ground truth odometry from vehicle",
    )
    # Add the launch argument to the launch description
    ld.add_action(topic_name_arg)
    # Create the node
    dvl_simulator_package_node = Node(
        package="dvl_simulator_package",
        namespace="/auv/gnc/navigation_sensors/dvl",
        executable="dvl_simulator_package_node",
        name="dvl_simulator_node",
        output="screen",
        parameters=[config, {"topic_name": LaunchConfiguration("topic_name")}],
    )

    ld.add_action(dvl_simulator_package_node)

    return ld
