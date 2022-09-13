#!/usr/bin/env bash

SCRIPT_DIR="$( dirname "$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )" )"

if [ -f "${SCRIPT_DIR}/CMakeLists.ROS2.txt" ]; then

	echo "Converting to ROS 2"
	mv "${SCRIPT_DIR}/CMakeLists.txt" "${SCRIPT_DIR}/CMakeLists.ROS1.txt"
	mv "${SCRIPT_DIR}/package.xml" "${SCRIPT_DIR}/package.ROS1.xml"
	mv "${SCRIPT_DIR}/CMakeLists.ROS2.txt" "${SCRIPT_DIR}/CMakeLists.txt"
	mv "${SCRIPT_DIR}/package.ROS2.xml" "${SCRIPT_DIR}/package.xml"

elif [ -f "${SCRIPT_DIR}/CMakeLists.ROS1.txt" ]; then

	echo "Converting to ROS 1"
	mv "${SCRIPT_DIR}/CMakeLists.txt" "${SCRIPT_DIR}/CMakeLists.ROS2.txt"
	mv "${SCRIPT_DIR}/package.xml" "${SCRIPT_DIR}/package.ROS2.xml"
	mv "${SCRIPT_DIR}/CMakeLists.ROS1.txt" "${SCRIPT_DIR}/CMakeLists.txt"
	mv "${SCRIPT_DIR}/package.ROS1.xml" "${SCRIPT_DIR}/package.xml"

else

	echo "Could not find CMakeLists.[ROS1/2].txt"
	return 1

fi
