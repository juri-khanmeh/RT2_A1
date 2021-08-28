#include "ros/ros.h"
#include "rt2_assignment1/Command.h"

#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/PositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   actionlib::SimpleActionClient<action::PositionAction> ac("goToPoint", true);
   
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
   		client_rp.call(rp);
   		p.target_pose.header.frame_id = "base_link";
		p.target_pose.header.stamp = ros::Time::now();
   		p.target_pose.pose.position.x = rp.response.x;
   		p.target_pose.pose.position.y = rp.response.y;
   		p.target_pose.pose.orientation.w = rp.response.theta;
   		
   		ROS_INFO("Sending goal");
   		ac.sendGoal(p);
   		std::cout << "\nGoing to the position: x= " << p.target_pose.pose.position.x << " y= " <<p.target_pose.pose.position.y << " theta = " <<p.target_pose.pose.orientation.w << std::endl;
   		
   		while(start){
   			//ac.waitForResult();
   			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
   				ROS_INFO("Target reached!");
   				break;   			
   			}
   			//else
   			//	ROS_INFO("The base failed to reach the target for some reason");   		
   		}

  
	} 
   	else
   		ac.cancelGoal();
   		ROS_INFO("goal is cancelled");
   	}
   }
   return 0;
}
