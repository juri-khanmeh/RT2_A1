## @package rt2_assignment1
# \file go_to_point.py
# \brief This file contains the node for controlling the motion of robot in the package rt2_assignment1
# \author Juri Khanmeh
# \version 0.1
# \date 25/09/2021
#
# \details
#
# Subscribes to: <BR>
# ° /odom
#
# Publishes to: <BR>
# ° /cmd_vel
#
# Service : <BR>
# ° /go_to_point
#
# Description :
#
# This node controls the motion of the robot. 
# In order to reach a position there are 3 states:
# 1. fix_yaw
# 2. go_straight_ahead
# 3. fix_final_yaw

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations

import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
position_ = Point()
desired_position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# action_server
act_s = None

##
# \brief '/odom' subscribtion callback.
# \param msg an odometry variable which contains the position and the orientation.
# \return the current position_ and orientation of the robot.
#
# This function return a position and yaw
# after converting the orientation angles from quaternion to euler.
#
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
# \brief robot's state changer
# \param state is the integer number which represents the new state.
# \param state_ previous state.
# \return print a message announcing the current state.
#
# This function changes the state variable and confirms it in a message.
#
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
# \brief angle normalizer function
# \param angle is the angle which we want to normalize it.
# \return the normalized angle.
#
# This function normalize the angle which is given in order to 
# use the angle in the following process without errors.
#
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
# \brief fixing robot's yaw function
# \param des_pos is destination position of the robot.
# \return NULL.
#
# This function fixes the yaw of the robot according to the destination position.
# So the robot can go straight to the destination point.
#
def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(1)

##
# \brief straight motion function
# \param des_pos is the destination position.
# \return NULL
#
# This function is for making the robot move straight towards the destination.
# But also it sets some angular velocity according the yaw error.
#
def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
# \brief fixing final orientation function
# \param des_yaw is the target yaw.
# \return NULL
#
# This function is for controlling the final orientation of the robot.
# The angular velocity is set according to the yaw error.
#
def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(3)

##
# \brief last step function
#
# This function sets all the robot's velocities to zero
# for making the robot stop after reaching the requested target.       

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)

##
# \brief motion control function
# \param goal is the target position that the robot must reach.
# \return feedback stats.
#
# This function changes the robot state according to 
# the current position of the robot and the target position.
# The objective of this function is to make the robot 
# reach a specific point.
#    
def go_to_point(goal):
    global state_, desired_position_, position_
    global act_s
	
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.orientation.w
	
    state_ = 0
    rate = rospy.Rate(20)
    success = True

    feedback = rt2_assignment1.msg.PositionFeedback()
    result = rt2_assignment1.msg.PositionResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
        	rospy.loginfo('Goal was preempted')
        	act_s.set_preempted()
        	done()
        	success = False
        	break
        elif state_ == 0:
        	feedback.stat = "Fixing the yaw"
        	feedback.actual_pose.position = position_
        	act_s.publish_feedback(feedback)
        	fix_yaw(desired_position_)
        elif state_ == 1:
        	feedback.stat = "Angle aligned"
        	feedback.actual_pose.position = position_
        	act_s.publish_feedback(feedback)
        	go_straight_ahead(desired_position_)
        elif state_ == 2:
        	feedback.stat = "Position reached"
        	feedback.actual_pose.position = position_
        	act_s.publish_feedback(feedback)
        	fix_final_yaw(des_yaw)
        elif state_ == 3:
        	feedback.stat = "Target reached!"
        	feedback.actual_pose.position = position_
        	act_s.publish_feedback(feedback)
        	done()
        	break
        else:
        	rospy.logerr('Unknown state!')
        		
        rate.sleep()
            
    if success:
    	rospy.loginfo('Goal: Succeeded!')
    	act_s.set_succeeded(result)
##
# \brief main function
# \param null
# \return null
#
# This is the main function of go_to_point node
# It initializes the node, creates a '/cmd_vel' publisher, '/odom' subscriber,
# '/go_to_point' simple action server.
#
def main():
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

	#define an action server instead of simple server
    act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.PositionAction, go_to_point, auto_start=False)
    act_s.start()
	
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
