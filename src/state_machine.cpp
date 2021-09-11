
#include <chrono>
#include <functional>
#include <memory>
#include <cinttypes>
#include <string>


#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp" 
#include "rt2_assignment1/srv/random_position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
class StateMachine : public rclcpp::Node
{
public:
  StateMachine(const rclcpp::NodeOptions & options)
  : Node("StateMachine_server",options)
  {
    start = false; //represent the input command
    finish = true; //indicator for reaching the goal
    
    //Command server
    service_ = this->create_service<rt2_assignment1::srv::Command>("/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
    
    //Random Position client
    client1_ = this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server");
    while (!client1_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "client_random interrupted while waiting for service to appear.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for RandomPosition service to appear...");
    }
    
    //GoToPoint client
    client2_ = this->create_client<rt2_assignment1::srv::Position>("go_to_point");
    while (!client2_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "client_go interrupted while waiting for service to appear.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for go_to_point service to appear...");
    }    
    
    //Defining variables
	request1 = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
	response1 = std::make_shared<rt2_assignment1::srv::RandomPosition::Response>(); 
	request2 = std::make_shared<rt2_assignment1::srv::Position::Request>();
	
  	request1->x_max= 5.0;
  	request1->x_min= -5.0;
  	request1->y_max= 5.0;
  	request1->y_min= -5.0;

	//timer which executed regularly to check the status
	timer_ = this->create_wall_timer(2000ms,std::bind(&StateMachine::status_check, this));	
    
  }
  
private:
    //timer call back
    void status_check(){
    	if(start && finish){
    		finish = false;
    		call_client2();	
    	}
    }
  	
  	//random position call back
  	void call_client1(){

	  	finish = false;
  		using ServiceResponseFuture = rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture;
		auto response_received_callback = [this](ServiceResponseFuture future) {
	  		response1 = future.get();	  		
	  		RCLCPP_INFO(this->get_logger(), "Going to the position: x= %f y= %f theta= %f",request2->x, request2->y, request2->theta);
	  		
  		};
    	auto result_future = client1_->async_send_request(request1, response_received_callback);
	}
	
	//GoToPoint client call back
	void call_client2()
	{
		call_client1();
		
    	request2->x= response1->x;
  		request2->y= response1->y;
  		request2->theta = response1->theta;
  		using ServiceResponseFuture = rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture;
  		auto goal_finish_callback = [this](ServiceResponseFuture future) {
  			RCLCPP_INFO(this->get_logger(), "Request: x= %f y= %f theta= %f",request2->x, request2->y, request2->theta);
  			RCLCPP_INFO(this->get_logger(), "Mission Complete!");
  			finish = true;
  		};

    	auto future_result = client2_->async_send_request(request2, goal_finish_callback);  
    	request2->x= response1->x;
  		request2->y= response1->y;
  		request2->theta = response1->theta; 

    }   

    //User Interface service call back
    void user_interface(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<rt2_assignment1::srv::Command::Request> request,
    const std::shared_ptr<rt2_assignment1::srv::Command::Response> response)
    {
    	(void)request_header;
      	if (request->command == "start"){
  	    	start = true;
      	}
      	else {
  	    	start = false;
        	}
      	response->ok = true;
      	RCLCPP_INFO(this->get_logger(), "Command received...");
    }
    

    rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service_;
    rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr client1_;
    rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client2_;
    
    rclcpp::TimerBase::SharedPtr timer_;
  
    std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> request1;    
    std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response1;  			
    std::shared_ptr<rt2_assignment1::srv::Position::Request> request2;    
    
    bool start;
    bool finish;
  
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine) 

