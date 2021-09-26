## @package rt2_assignment1
# \file user_interface.py
# \brief This file contains the user interface of the package rt2_assignment1
# \author Juri Khanmeh
# \version 0.1
# \date 25/09/2021
#
# \details
#
# Service : <BR>
# ° /user_interface
#
# Description :
#
# This node represents the user interface for the package rt2_assignment1

import rospy
import time
from rt2_assignment1.srv import Command

##
# \brief main function
# \param null
# \return null
#
# This is the main function of user_interface.
# First, it initializes the node and creates a '/user_interface' service client.
# In the main loop, it receives an input (int) from the user.
# If the input is '1', it calls the '/user_interface' by sending "start" request.
# otherwise it calls the '/user_interface' by sending "stop" request.
#
def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
        	#print("Robot has started to move")
        	ui_client("start")
        	x = int(input("\nPress 0 to stop the robot "))
        else:
        	#print("Robot has stopped")
        	ui_client("stop")
        	x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
