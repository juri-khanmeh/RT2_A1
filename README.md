# Research Track 2 assignment 1 - ACTION + Jupyter

The Branch action contains the same package in ROS, but with the go_to_point node modelled as a "ROS action" server, instead of a "simple" server.
Given that, the robot FSM node should now implement mechanisms for possibly cancelling the goal, when the related user command is received.

## Description (modifications of the package)
### ACTION server
* We created an action message `Position.action` which consists of a goal, feedback and result.
```
#goal
geometry_msgs/PoseStamped target_pose
---
#result
bool ok
---
#feedback
geometry_msgs/Pose actual_pose
string stat
```
### (user_interface)
* We removed the message 
```
print("Please wait, the robot is going to stop when the position will be reached")
```
because now the robot must stop immediately once it receives the stop command.

### (go_to_point)
* We defined a simple action server instead of a simple server for `/go_to_point` topic
* So instead of using the request and the response we replaced them by a goal, a feedback and a result.
* Besides, now there is the posibility to cancel the goal before reaching the target.

### (state_machine)
* We created a simple action client instead of the simple client.
* We added a case for cancelling the goal.

## Instruction how to run the code:

1. Open a terminal (ROS sourced) then execute
```
roslaunch rt2_assignment1 sim.launch
```


