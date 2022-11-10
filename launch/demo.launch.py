import os

from datetime import datetime

from xacro import process_file
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
	# Launch description
	ld = LaunchDescription()

	# Launch arguments
	dry_run_arg = DeclareLaunchArgument(
		"dry_run", default_value=TextSubstitution(text="0"),
		description="Set to '1' to run CORC without LabVIEW heartbeat"
	)
	ld.add_action(dry_run_arg)

	# Package share path
	corc_path = get_package_share_directory("corc")
	x2_description_path = get_package_share_directory("x2_description")

	# Package share paths
	exo_path = os.path.join(corc_path, "config", "exo_default.yaml")
	gait_path = os.path.join(corc_path, "gaits", "walking.csv")
	rviz_path = os.path.join(x2_description_path, "rviz", "x2.rviz")

	# Process XACRO
	urdf_xacro = process_file(
		os.path.join(x2_description_path, "urdf", "x2_fixed_base.urdf.xacro")
	)

	# Nodes
	exo_node = Node(
		package="corc",
		executable="ExoApp",
		arguments=["-can", "can0"],
		name="exo",
		output="screen",
        parameters=[
			{
				"dry_run"   : LaunchConfiguration("dry_run"),
				"exo_path"	: exo_path,
				"gait_path" : gait_path
			},
			exo_path
		]
	)
	exo_splitter_node = Node(
        package="exo_splitter",
        executable="exo_splitter",
        name="exo_splitter"
    )
	robot_state_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		parameters=[
			{ "robot_description": urdf_xacro.toprettyxml(indent='	') }
		]
	)
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		arguments=["-d", rviz_path],
		name="rviz2"
	)
	ld.add_action(exo_node)
	ld.add_action(exo_splitter_node)
	ld.add_action(robot_state_node)
	ld.add_action(rviz_node)

	# Bag
	bag_path = f"{os.path.expanduser('~')}/nrm-logs/{datetime.now().strftime('%H%M_%d-%m-%Y')}"

	if os.path.exists(bag_path):
		os.remove(os.path.join(bag_path, "metadata.yaml"))
		os.remove(os.path.join(bag_path, f"{datetime.now().strftime('%H%M_%d-%m-%Y')}_0.db3"))
		os.removedirs(bag_path)

	ros2bag = ExecuteProcess(
		cmd=["ros2", "bag", "record", "-a", "-o", bag_path],
		output="screen"
	)
	ld.add_action(ros2bag)

	# Launch description
	return ld
