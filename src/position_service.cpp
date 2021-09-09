
#include <chrono>
#include <functional>
#include <memory>
#include <inttypes.h>
#include <string>
#include <math.h>


#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
class RandomServer : public rclcpp::Node
{
public:
  RandomServer(const rclcpp::NodeOptions & options)
  : Node("RandomPose_server",options)
  {
    service_ = this->create_service<rt2_assignment1::srv::RandomPosition>(
      "/position_server", std::bind(&RandomServer::handle_service, this, _1, _2, _3));
    
  }

private:

  double randMToN(double M, double N){     
  	return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
  }

  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response)
  {
  (void)request_header;
  response->x = randMToN(request->x_min, request->x_max);
  response->y = randMToN(request->y_min, request->y_max);
  response->theta = randMToN(-3.14, 3.14);
  
  }  
  rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr service_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RandomServer) 


