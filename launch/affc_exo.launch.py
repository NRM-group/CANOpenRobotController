import os

from datetime import datetime

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

	# Package share files
	exo_file = os.path.join(corc_path, "config", "exo_default_affc.yaml")
	gait_file = os.path.join(corc_path, "gaits", "walking.csv")
	affc_file = f"{os.path.expanduser('~')}/nrm-affc/affc_learned_parameters.yaml"

	# Nodes
	exo_node = Node(
		package="corc",
		executable="ExoApp",
		arguments=["-can", "can0"],
		name="exo",
		output="screen",
        parameters=[
			{
				"dry_run"	: LaunchConfiguration("dry_run"),
				"exo_file"	: exo_file,
				"gait_file" : gait_file,
				"affc_file"	: affc_file
			},
			exo_file,
			affc_file,
		]
	)
	exo_splitter_node = Node(
        package="exo_splitter",
        executable="exo_splitter",
        name="exo_splitter"
    )
	ld.add_action(exo_node)
	ld.add_action(exo_splitter_node)

	# Bag
	bag_path = f"{os.path.expanduser('~')}/nrm-logs/{datetime.now().strftime('%H%M_%d-%m-%Y')}"

	if os.path.exists(bag_path):
		os.remove(bag_path)

	ros2bag = ExecuteProcess(
		cmd=["ros2", "bag", "record", "-a", "-o", bag_path],
		output="screen"
	)
	ld.add_action(ros2bag)

	# Launch description
	return ld