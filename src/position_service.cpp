/**
* \file position_service.cpp
* \brief This file defines the position service
* \author Juri Khanmeh
* \version 0.1
* \date 25/09/2021
*
* \details
*
* Services : <BR>
* ° /position_server
*
* Description :
*
* This node is a random position server. It responds with a random position (x,y,theta), which is used as a target to be reached.
* The random number for (x,y) position is between [min, max] that can be set in the request.
* The [min, max] values of random number for (theta) is initially set and fixed [-3.14, 3.14]
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
* \brief random number generator
* \param M defines the minimum possible value of the random number
* \param N defines the maximum possible value of the random number
* \return the random position (x,y,theta).
*
* randMToN is a function for generating random position in given range [min,max].
*/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
* \brief '/position_server' server callback
* \param &req defines the lower and upper bounds for x and y random numbers' range
* \param &res defines the position [x,y] and the orientation [theta]
* \return always true as this method cannot fail.
*
* This function creates a RandomPosition message. It fills up the response message
* with random value for x, y and theta
*/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
* \brief main function
* \param argc 
* \param argv
* \return always true.
*
* The main function initializes the node and creates a '/position_server' service server.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
