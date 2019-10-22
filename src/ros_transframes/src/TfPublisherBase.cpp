/*
 * Copyright (c) ??? - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Authors: Pavel Shumejko
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include <thread>
#include <stdexcept>

/**
 * This class creates a publisher that publishes and updates the geometry_msgs::TransformStamped messages 
 *
 */
class TfPublisherBase
{

public:
    //! @name global variables
    //@{
    ros::NodeHandle g_n;
    geometry_msgs::TransformStamped g_tfs;
    ros::Publisher g_tf_pub;
    //@}

    /**
     * Constructor
     * publisher is created and geometry_msgs::TransformStamped message is initialized
     */
    TfPublisherBase() 
    {
        g_tf_pub = g_n.advertise<geometry_msgs::TransformStamped>("world/base_tf_enu", 1);

        g_tfs.header.stamp = ros::Time::now();
        g_tfs.header.frame_id = "world";
        g_tfs.child_frame_id = "world/base_tf_enu";

        g_tfs.transform.translation.x = 0.0;
        g_tfs.transform.translation.y = 0.0;
        g_tfs.transform.translation.z = 0.0;
        g_tfs.transform.rotation.x = 0.0;
        g_tfs.transform.rotation.y = 0.0;
        g_tfs.transform.rotation.z = 0.0;
        g_tfs.transform.rotation.w = 1.0;
    }

    /**
     * Function that converts the input string into 6 numbers and updates the geometry_msgs::TransformStamped message
     */
    void processInput(const std::string& input);
 
    /**
     * Thread function that publishes conctantly on the "world/base_tf_enu" topic in the background
     */
    void threadPublish();
};

void TfPublisherBase::processInput(const std::string& input)
{
    ROS_INFO("You entered:");
    std::cout << input << std::endl;

    std::vector<float> vect;
    std::stringstream ss(input);

    float i;    // float variable to store the parsed numbers

    while (ss >> i)
    {
        vect.push_back(i);

        if (ss.peek() == ',')
             ss.ignore();
    }
	
    if (vect.size() < 6)
        throw std::runtime_error("Wrong vector size");
    
    // print the parsed values
    std::cout << " "<<std::endl;
    ROS_INFO("Parsed:");
    std::cout << "X:";
    std::cout << vect[0]<<std::endl;
    std::cout << "Y:";
    std::cout << vect[1]<<std::endl;
    std::cout << "Z:";
    std::cout << vect[2]<<std::endl;
    std::cout << "ROLL:";
    std::cout << vect[3]<<std::endl;
    std::cout << "PITCH:";
    std::cout << vect[4]<<std::endl;
    std::cout << "YAW:";
    std::cout << vect[5]<<std::endl;
    std::cout << " "<<std::endl; 

    // update the msg
    g_tfs.transform.translation.x = vect[0];
    g_tfs.transform.translation.y = vect[1];
    g_tfs.transform.translation.z = vect[2];
    g_tfs.transform.rotation.x = vect[3];
    g_tfs.transform.rotation.y = vect[4];
    g_tfs.transform.rotation.z = vect[5];
    g_tfs.transform.rotation.w = 1;
   
}

void TfPublisherBase::threadPublish()
{
    ros::Rate r_hz(1.0);
    while(1)
    {	
        g_tf_pub.publish(g_tfs);
	r_hz.sleep();
    }
}

int main(int argc, char** argv)
{
    // rosnode initialization 
    ros::init(argc, argv, "tf_publisher_base");

    // Create the object
    TfPublisherBase tpb;

    // grab the node handle from the class
    ros::NodeHandle nh = tpb.g_n;

    ROS_INFO("Node started");
  
    std::string input_str;   

    // start the thread function from the class
    // NOTE: For some reason I could not start the thread (call the function) in the consturctor
    std::thread t(&TfPublisherBase::threadPublish,&tpb);
    t.detach();

    ros::Rate rate_hz(10.0);
    while (nh.ok())
    {
        ROS_INFO("Awaiting user input of 6 numbers divided with ',': ");
  	std::cin >> input_str;
	tpb.processInput(input_str);    // calls the class function to process the input string
  	        
        rate_hz.sleep();
    }
    return 0;
};
