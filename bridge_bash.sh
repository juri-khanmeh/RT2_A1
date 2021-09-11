#!/bin/bash

gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore"
gnome-terminal --tab --title="ros" -- bash -c "sleep 1; source ros.sh; roslaunch rt2_assignment1 sim_ros2.launch"
gnome-terminal --tab --title="bridge" -- bash -c "source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2" -- bash -c "source ros2.sh; ros2 launch rt2_assignment1 cpp_launch.py"
