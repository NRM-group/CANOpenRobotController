import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

	# Launch arguments
	dry_run_arg = DeclareLaunchArgument(
		"dry_run", default_value=TextSubstitution(text="0"),
		description="Set to `1` to run CORC without LabVIEW heartbeat"
	)

	# Package share path
	corc_path = get_package_share_directory("corc")

	# Package share files
	exo_file = os.path.join(corc_path, "config", "exo_default.yaml")
	gait_file = os.path.join(corc_path, "gaits", "walking.csv")

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
				"exo_file"	: exo_file,
				"gait_file" : gait_file
			},
			exo_file
		]
	)
	exo_splitter_node = Node(
        package="exo_splitter",
        executable="exo_splitter",
        name="exo_splitter"
    )

	return LaunchDescription([
		dry_run_arg,
		exo_node,
		exo_splitter_node
	])
