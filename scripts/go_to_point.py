"""
.. module:: go_to_point
   :platform: Unix
   :synopsis: Python module for piloting the robot to the target
   
.. moduleauthor:: Juri Khanmeh

ROS node for driving a robot to a specific point

Subscribes to:
	/odom topic where the simulator publishes the robot position
	
Publishes to:
	/cmd_vel the desired robot position
	
Service :
	/go_to_point to start the robot motion.
"""

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
"""Point: actual robot position

"""
desired_position_ = Point()
"""Point: target position

"""
yaw_ = 0
"""Float: robot orientation

"""
position_ = 0
state_ = 0
"""Int: robot motion state number

"""
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


def clbk_odom(msg):
    """
    Function for '/odom' subscription callback.
	
    Args:
		msg(Odometry): the current odometry of the robot.
		
    """
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


def change_state(state):
    """
    Function for changing the robot's state.
	
    Args:
		state(Int): the new state number	
		
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    Function for normalizing the angle between -pi and pi.
	
    Args:
		angle(Float): the input angle.
		
    Returns:
		angle(Float): the normalized angle.
		
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    """
    Function for fixing robot's yaw into the direction of the destination position.
	
    Args:
		des_pos(Point): the destination position.
		
    """
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


def go_straight_ahead(des_pos):
    """
    Function to move the robot straight to the destination point.
	
    Args:
		des_pos(Point): the destination position.

    """
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


def fix_final_yaw(des_yaw):
    """
    Function for fixing the final yaw of the robot.
	
    Args:
		des_yaw(Float): the target yaw.

    """
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
   

def done():
    """
    Function for setting all the robot's velocities to zero.
	
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)

  
def go_to_point(goal):
    """
    Function to move the robot to a specific point.
	
    Args:
		goal(PositionAction): PositionAction message which has the target position.
		
    """
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


def main():
    """
    main function for the node 'go_to_point'
		
    """
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
