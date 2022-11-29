# CAN Open Robot Controller (CORC)

This branch of the CAN Open Robot Controller is the specific implementation of the NRM group's integration and software architecture. It adds two new dependencies on top of the existing libraries in CORC, both also developed by the NRM group.

- [`controllerlib`](https://github.com/NRM-group/controllerlib)
- [`trajectorylib`](https://github.com/NRM-group/trajectorylib)

In addition, this version of CORC integrates ROS2 Foxy as part of the communication structure with LabVIEW over LAN. As such, this branch may not be independently run outside of ROS2.

The `nrm-release` branch utilises basic controllers such as friction and compensation control. The [`nrm-affc`](https://github.com/NRM-group/CANOpenRobotController/tree/nrm-affc) branch replaces these controllers with an adaptive feed-forward controller which improves overall performance. Please see [`nrm-affc`](https://github.com/NRM-group/CANOpenRobotController/tree/nrm-affc) for more advanced use cases.

## Install

Clone the repositories in the ROS workspace.

```sh
cd ~/ros2_ws/src/
git clone --recurse-submodules -b nrm-release https://github.com/NRM-group/CANOpenRobotController
git clone --recurse-submodules https://github.com/NRM-group/Ai-ExoMotus
```

Build the custom `exo_msgs` first, source it, then build the entire workspace and install.

NOTE: Edit `CMakeLists.txt` under `CANOpenRobotController` for specific configurations, or leave as default.

```sh
cd ~/ros2_ws
colcon build --packages-select exo_msgs
source install/setup.bash
colcon build
source install/setup.bash
```

## Usage

Run the CAN initialiser script if starting since boot-up or the cable was disconnected.

```sh
ros2 run corc initCAN0.sh
```

Launch the CORC app. You may observe available arguements with the following command.

```sh
ros2 launch corc exo.launch.py -s
```

As of current, the only argument available is the `dry_run` argument. Set this to 1 to run CORC without needing connection with LabVIEW. This is 0 by default.

```sh
ros2 launch corc exo.launch.py dry_run:=1 # or dry_run:=0 (default)
```

If running CORC without LabVIEW, the following can be run in a separate terminal to provide a GUI for control using Qt.

NOTE: New terminals must source the install setup script.

```sh
source ~/ros2_ws/install/setup.bash
ros2 launch exo_launch dev.launch.py
```