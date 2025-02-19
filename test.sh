#!/bin/bash

# Source ROS 2 environment
source ~/robot_ws/install/local_setup.bash

# Launch each file in a new terminal window
gnome-terminal -- bash -c "ros2 launch tracer_base tracer_base.launch.py; exec bash"
#gnome-terminal -- bash -c "ros2 launch tracer_base tracer_mini_base.launch.py; exec bash"
#gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
gnome-terminal -- bash -c "ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py; exec bash"
gnome-terminal -- bash -c "ros2 launch rplidar_ros rplidar_a3_launch.py; exec bash"
gnome-terminal -- bash -c "ros2 launch tracer_base sensor.xml; exec bash"
gnome-terminal -- bash -c "ros2 launch tracer_base rtb.xml; exec bash"
gnome-terminal -- bash -c "ros2 launch tracer_base nav.xml; exec bash"

