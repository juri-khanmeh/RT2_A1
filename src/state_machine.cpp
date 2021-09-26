/**
* \file state_machine.cpp
* \brief state machine for rt2_assignment1 control
* \author Juri Khanmeh
* \version 0.1
* \date 25/09/2021
*
* \details
*
* Services : <BR>
* ° /user_interface
* ° /position_server
* ° /go_to_point
*
* Description :
*
* This file defines the state machine of rt2_assignment1 control. 
* Once this node receives a command to start the random movement, it calls '/position_server'.
* The server '/position_server' returns a random position. 
* Once the node receives the random position, it calls the '/go_to_point' with target request.
* If the node receives a command to stop the random movement, it cancels the goal.
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/PositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false; ///< start flag
bool cancel = true; ///< cancel goal flag
bool n_target = true; ///< new target flag

/**
* \brief server callback of '/user_interface'.
* \param &req is Command::Request which is the command ["start"/"stop"].
* \param &res is Command::Response is a boolean 'True' when the request is received. 
* \return always true as this method cannot fail.
*
* This function sets the global variable 'start' to true if the request was "start"
* otherwise it sets it to false.
*/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node, creates a '/user_interface' service server, 
* '/position_server' service client and '/go_to_point' simple action client.
* Then in the main while loop, it checks whether the command is "start" or "stop".
* If it is "start", it calls '/position_server' to get a random position.
* Then it sends this random position as a target request to '/go_to_point' action server.
* If the goal is reached, it repeats the same process again.
* If the goal was interupted it stops the process and waits for the new command
* to turn to "start" again.
*/
int main(int argc, char **argv)
{
   //initialize nodes
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   actionlib::SimpleActionClient<rt2_assignment1::PositionAction> ac("/go_to_point", true);
   
   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
   	ROS_INFO("Waiting for the move_base action server to come up");
  }
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::PositionGoal p;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		if(n_target){
   			client_rp.call(rp);
   			p.target_pose.header.frame_id = "base_link";
			p.target_pose.header.stamp = ros::Time::now(); 
			//setting the position which received from random server
   			p.target_pose.pose.position.x = rp.response.x;
   			p.target_pose.pose.position.y = rp.response.y;
   			p.target_pose.pose.orientation.w = rp.response.theta;
   		
   			cancel = false;
   			n_target = false;
   			ac.sendGoal(p);
   			std::cout << "\nGoing to the position: x= " << p.target_pose.pose.position.x << " y= " <<p.target_pose.pose.position.y << " theta = " <<p.target_pose.pose.orientation.w << std::endl;
   		}
   		else{
   			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
   				n_target = true;
   			}	
   		}	
   }
   
   	else
   		if(!cancel){
   			ac.cancelGoal();
   			cancel = true; n_target = true;
   		}
   		
   	}
   
   return 0;
}
