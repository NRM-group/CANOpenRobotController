import os

from xacro import process_file
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch_ros.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()

	# Package share path
	corc_path = get_package_share_directory("CORC")
	x2_description_path = get_package_share_directory("x2_description")

	# Package share files
	x2_file = os.path.join(corc_path, "config", "x2_params.yaml")
	gait_file = os.path.join(corc_path, "gaits", "walking.csv")
	rviz_config = os.path.join(x2_description_path, "rviz", "x2.rviz")
	urdf_file = process_file(
		os.path.join(x2_description_path, "urdf", "x2_ros2_control.urdf.xacro")
	)

	# Nodes
	x2_node = Node(
		package="CORC",
		executable="X2Machine_APP",
		arguments=["-can", "vcan0"],
		name="x2",
		output="screen",
        parameters=[
            { "walking_gait" : gait_file },
            { "x2_params"    : x2_file   }
		]
	)
	x2_ik_node = Node(
		package="CORC",
		executable="x2_ik"
	)
	robot_state_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		parameters=[
			{ "robot_description": urdf_file.toprettyxml(indent='	') }
		]
	)
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		arguments=["-d", rviz_config],
		name="rviz2"
	)

	##################### GAZEBO STUFF ###################
	# spawn_entity = Node(
	# 	package="gazebo_ros",
	# 	executable="spawn_entity.py",
	# 	name="urdf_spawner",
	# 	output="screen",
	# 	arguments=["-topic", "/robot_description", "-entity", "x2"]
	# )

	# gazebo = IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource([os.path.join(
	# 			get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
	# 		)
	
	ld.add_action(x2_node)
	ld.add_action(x2_ik_node)
	ld.add_action(robot_state_node)
	ld.add_action(rviz_node)
	return ld
