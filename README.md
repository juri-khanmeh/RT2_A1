# Research Track 2 assignment 1 - ACTION + Jupyter Notebook

The Branch action contains the same package in ACTION, but with some modification for adapting some changes in the new interface in Jupyter Notebook.

Now the user interface is enhanced and besides, we showcase some graphs and plots which are usefull for tracking some information about the robots.

## Description (Jupyter Notebooks)
### better_interface
* We created a notebook with a name `better_interface` which consists of some useful interface for controlling the robot.
- starting / stopping the robot's random position behaviour by using two Buttons
- setting the linear and angular velocity by using two Sliders
- directly controlling the robot movements by using 5 Buttons

### graphs and plot
* We created a notebook with a name `graphs and plot` which has some graphs and plots for position, velocity, goals information.
- a line plot for visualizing cmd_vel vs. actual velocity (for linear and angular velocity)
- a bar plot displaying the number of reached targets and cancelled targets
- a hist plot showing the time required to reach targets
- an xy graph showing the robot's position and the orientation

### Histogram
* We created a separate notebook for histogram graph `Histogram`, just because this plot slow down the execution of other plots. But we can merge the three notebooks in one if we want.


## Instruction how to run the code:

1. Open a terminal (ROS sourced) then execute
```
roslaunch rt2_assignment1 sim.launch
```
2. Open Jupyter notebooks
3. Run the codes.

Note: we kept the old interfaces in command line.

