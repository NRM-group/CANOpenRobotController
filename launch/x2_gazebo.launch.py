# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    #Directory Locations    
    corc_path = get_package_share_directory("CORC")
    # x2_description_path = get_package_share_directory("x2_description")



    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py', ]),
            )

    ##Loading files
    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('gazebo_ros2_control_demos'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'test_cart_position.xacro.urdf')
    x_doc = os.path.join(get_package_share_directory("x2_description"), "urdf", "x2_ros2_control.urdf.xacro")
    
    doc = xacro.parse(open(x_doc))
    xacro.process_doc(doc)
    # xacro_file = xacro.process_file(os.path.join(get_package_share_directory("x2_description"), 'urdf', 'x2_ros2_control.urdf.xacro'))
    rviz_config = os.path.join(get_package_share_directory("x2_description"), 'rviz', 'x2.rviz')
    gait_file = os.path.join(corc_path, "gaits", "walking.csv")
    x2_file = os.path.join(corc_path, "config", "x2_params.yaml")
    # corc_app = 
    

    # params = {'robot_description': xacro_file.toprettyxml(indent='	') }
    params = {'robot_description': doc.toxml()}
    

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		arguments=["-d", rviz_config],
		name="rviz2"
	)
    x2_node = Node(
		package="CORC",
		executable="X2Machine_APP",
		arguments=["-can", "vcan0"],
		name="x2",
		output="screen",
        parameters=[
            { "walking_gait" : gait_file },
            { "x2_params"    : x2_file   },
			x2_file
		]
	)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'x2'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        # gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz_node,
        # x2_node,
    ])