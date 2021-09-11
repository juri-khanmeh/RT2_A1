# Research Track 2 assignment 1 - ROS2

In the branch ros2, the cpp nodes (Robot FSM and position server) are written for ROS2 as components, so that by using the ros1_bridge, they communicate with the ROS nodes and with the simulation in Gazebo. 

## Description (Content of the package)
### ROS Nodes
Written in py as individuale nodes
1. (user_interface)
* ROS client: it calls `Command` service through `/command` topic.
2. (go_to_point)
* ROS server: `Position` server, accepts call through `/go_to_point` topic, sets a position for a robot and make it reach this target.
* ROS publisher: it publsihes robot's velocities to `/cmd_vel` topic.
* ROS subscriber: it subscribes to `/odom` topic in order to obtain robot's position.
### ROS2 Nodes
Written in cpp as components
1. (position_service)
* ROS2 server: it receives calls through `/random_position` topic, generates a random position.
2. (state_machine)
* ROS2 server: it receives calls from (user_interface) through `/command` topic, in order to start or stop the robot action.
* ROS2 client: it calls Position Server through `/random_position` topic, in order to receive a random positio. Also it calls GoToPoint Server through `/go_to_point` topic by sending a target position.

## Instruction how to run the code:

1. Open a 1st terminal (ROS sourced) then execute
```
roslaunch rt2_assignment1 sim_ros2.launch
```
2. Open a 2nd terminal (ROS and ROS2 sourced) then execute
```
ros2 run ros1_bridge dynamic_bridge
```
3. Open a 3rd terminal (ROS2 sourced) then execute
```
ros2 launch rt2_assignment1 cpp_launch.py
```
Alternatively, we can run the bash script `bridge_bash.sh` which include the three previous steps.
```
./bridge_bash.sh
```

