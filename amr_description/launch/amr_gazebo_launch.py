#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*  Filename:			amr_gazebo_launch.py
*  Description:         Use this file to spawn AMR inside amr arena world in the gazebo simulator and publish robot states.
*  Created:				08/01/202$
*  Last Modified:	    08/01/2024
*  Author:				Archit Jain
*  
*****************************************************************************************
'''

import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name='amr_description'

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package=pkg_name).find(pkg_name)

    xacro_file_amr = os.path.join(pkg_share, 'models/','amr/', 'amr.xacro')
    assert os.path.exists(xacro_file_amr), "The amr.xacro doesnt exist in "+str(xacro_file_amr)
    robot_description_config_amr = xacro.process_file(xacro_file_amr)
    robot_description = robot_description_config_amr.toxml()


    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_name), 'launch', 'start_world_launch.py'),
        )
    )

    robot_state_publisher_node_amr = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
    )

    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments = ["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen')
    
    spawn_amr = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='amr_spawner',
    	executable='spawn_entity.py',
        arguments=['-entity', 'amr', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
        output='screen'
    )

    joint_state_publisher_node_amr = launch_ros.actions.Node(
        package="joint_state_publisher",
        name="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    joint_state_publisher_gui_node_amr = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        robot_state_publisher_node_amr,
        # static_transform,
        spawn_amr,
        joint_state_publisher_node_amr,
        # joint_state_publisher_gui_node_amr
    ])