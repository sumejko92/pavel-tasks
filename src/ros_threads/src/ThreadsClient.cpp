/*
 * Copyright (c) ??? - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Authors: Pavel Shumejko
 */

#include "ros/ros.h"
#include "ros_threads/TimeDelay.h"
#include "random"
#include "string"
#include "thread"
#include "sstream"
#include <math.h> 

/**
 * This class creates a service client and contains the function which calls a service with a time delay as an argument and based on the response calculates the difference in milliseconds between the current time and service reply Time
 *
 */

class Client
{
    //! @name global variables
    //@{
    ros::ServiceClient service_client;    // service client
    //@}

public:
    Client();    // The constructor initializes the service client
    ros::NodeHandle n;    // Node handle

    /**
     * Thread function which runs alongside the main loop. It generates the random delay, calls the service and based on the response calculates the difference in miliseconds between the current time and service reply Time
     */
    void startThread();

};    // end of Client

Client::Client(void)
{
    service_client = n.serviceClient<ros_threads::TimeDelay>("/unix_time_now");    // client initialization
}

void Client::startThread()
{
    ros_threads::TimeDelay srv;

    double numeric;    // local variable to store a numeric value

    ros::Rate r(1.0);    // 1 Hz rate for 1 sec loop     
    while(1)
    {
        int random_num=rand()% 4 + 1;    // creates random integer in the range between 0 and 5	
	srv.request.Delay_s = random_num;     // initialize the request

        // if server is up call the service
        if (service_client.call(srv))
	{
            ros::Time current = ros::Time::now();    // grab current time

	    //convert the string response to int
	    std::stringstream(srv.response.Time)>>numeric;

	    double res,intpart;    // double variables
	    res = modf (numeric,&intpart);    // gets the fracpart of the numeric value
	    int difference = current.nsec/1000000 - static_cast<int>(res*1000);    // calculates the difference
	    ROS_INFO("Time Difference is : %d ms ",difference);
	}
	else
	{
	    ROS_ERROR("Failed to call service /unix_time_now");
        }
    } // end while
} 


int main(int argc, char **argv)
{
    // rosnode initialization  
    ros::init(argc, argv, "threads_client");

    // Create the object
    Client client;

    // grab the node handle from the class
    ros::NodeHandle nh = client.n;

    // start the thread
    std::thread t(&Client::startThread,&client);
    t.detach();

    ros::Rate rate_hz(2.0);    // 2 Hz rate for 0.5 sec loop
    while (nh.ok())
    {
        ros::Time time = ros::Time::now();    // grabs current time

	ROS_INFO("Current time=%d.%d", time.sec,time.nsec);    // printing of unix time up to nanoseconds 

	rate_hz.sleep();
	ros::spinOnce();
    }
 
    return 0;
}
