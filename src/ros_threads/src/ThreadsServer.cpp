/*
 * Copyright (c) ??? - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Authors: Pavel Shumejko
 */

#include "ros/ros.h"
#include "ros_threads/TimeDelay.h"
#include "string"
#include "thread"
#include "chrono"

/**
 * This class contains the functions for a ros node that advertise ROS service /unix_time_now that takes an integer Delay_s as request and replies with a string Time
 *
 */

class Server
{

public:
    Server() { }

    /**
     * Callback function for the server service which based on the service call argument (Delay_s) sends
     * back the response (unix time up to milisec) with or without a delay. To add a delay another
     * thread function (startThread) is called. 
     * @param req gets the request from the service call
     * @param res stores the response of the service
     */
    bool serviceCallback(ros_threads::TimeDelay::Request  &req,
             ros_threads::TimeDelay::Response &res);
    /**
     * Thread function to implement the delay and send back the response.
     * @param td gets the time delay
     * @param response gets the time when the service was called
     */
    void startThread(int td,std::string response);

};

void Server::startThread(int td,std::string response)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(td*1000)));    // adds delay
    ROS_INFO_STREAM("Sending back response AFTER DELAY: " + response);
}


bool Server::serviceCallback(ros_threads::TimeDelay::Request  &req,
    ros_threads::TimeDelay::Response &res)
{
    ros::Time result = ros::Time::now();    // grab the time when service was called
    int milisec = result.nsec/1000;    // get the ms
    res.Time = std::to_string(result.sec) + "." + std::to_string(milisec);    // create response of type string
  
    ROS_INFO("Receiving request: Delay=%ld", (long int)req.Delay_s);

    // Starts a thread or sends back the response instantly depending on the Delay_s value
    if(req.Delay_s > 0 && req.Delay_s < 3)
    {
        std::thread t(&Server::startThread,this,req.Delay_s,res.Time);    // calls the thread function
	t.detach();
    }
    else 
    {	
	ROS_INFO_STREAM("sending back response: " + res.Time);
    }

    return true;
}


int main(int argc, char **argv)
{
    // rosnode initialization  
    ros::init(argc, argv, "threads_server");
  
    // Create the handle
    ros::NodeHandle n;

    // Create the object
    Server server;

    // Creates the service which calls the class functions
    ros::ServiceServer server_service = n.advertiseService("/unix_time_now", &Server::serviceCallback,&server);
    
    ROS_INFO("Feel free to call the service");

    ros::Rate rate_hz(1.0);
    while (n.ok())
    {
        rate_hz.sleep();
	ros::spinOnce();
    }
 
    return 0;
}

