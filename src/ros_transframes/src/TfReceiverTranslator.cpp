/*
 * Copyright (c) ??? - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Authors: Pavel Shumejko
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

/**
 * This class contains the subscriber for enu and publishers for ned and nwu and does the neccesary transformaitons
 *
 */
class TfTranslator
{

public:
    //! @name global variables
    //@{
    ros::NodeHandle g_n;    // Node handle
    ros::Subscriber enu_sub;    // Subscriber to the world/base_tf_enu topic
    ros::Publisher ned_pub;    // Publisher for the world/tf_ned topic
    ros::Publisher nwu_pub;    // Publisher for the world/tf_nwu topic
    geometry_msgs::TransformStamped ned;    
    geometry_msgs::TransformStamped nwu;    
    //@}

     //Constructor. Initialization of the subscriber and publishers is done here
    TfTranslator()
    { 
        // Subscriber to the world/base_tf_enu topic
        enu_sub = g_n.subscribe("world/base_tf_enu", 1, &TfTranslator::enuCallback, this);

        // Initialization of the ned and nwu publishers
        ned_pub = g_n.advertise<geometry_msgs::TransformStamped>("world/tf_ned", 1);
        nwu_pub = g_n.advertise<geometry_msgs::TransformStamped>("world/tf_nwu", 1);
    }

    /**
     * Callback function for the subscriber. Transformation from enu to ned and nwu is done here
     */
    void enuCallback(const geometry_msgs::TransformStampedConstPtr& msg);
};


void TfTranslator::enuCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
    geometry_msgs::TransformStamped enu;

    //grab the enu msg
    enu=*msg;

    //convert enu to ned
    ned.header.stamp = ros::Time::now();
    ned.header.frame_id = "world";
    ned.child_frame_id = "world/world/tf_ned";

    ned.transform.translation.x = enu.transform.translation.y;	
    ned.transform.translation.y = enu.transform.translation.x;	
    ned.transform.translation.z = -enu.transform.translation.z;	

    ned.transform.rotation.x = enu.transform.rotation.y;	
    ned.transform.rotation.y = enu.transform.rotation.x;	
    ned.transform.rotation.z = -enu.transform.rotation.z;	
    ned.transform.rotation.w = enu.transform.rotation.w;	

    //convert enu to nwu
    nwu.header.stamp = ros::Time::now();
    nwu.header.frame_id = "world";
    nwu.child_frame_id = "world/world/tf_nwu";

    nwu.transform.translation.x = enu.transform.translation.y;	
    nwu.transform.translation.y = -enu.transform.translation.x;	
    nwu.transform.translation.z = enu.transform.translation.z;	

    nwu.transform.rotation.x = enu.transform.rotation.y;	
    nwu.transform.rotation.y = -enu.transform.rotation.x;	
    nwu.transform.rotation.z = enu.transform.rotation.z;	
    nwu.transform.rotation.w = enu.transform.rotation.w;	
}


int main(int argc, char** argv)
{

    // rosnode initialization 
    ros::init(argc, argv, "tf_receiver_translator");

    // Create the object
    TfTranslator translator;

    // node handle
    ros::NodeHandle nh = translator.g_n;

    /*
    * This chunk has been moved inside the constructor of the class
    *
        // Subscriber to the world/base_tf_enu topic
        //ros::Subscriber enu_sub = nh.subscribe("world/base_tf_enu", 1, &TfTranslator::enuCallback, &translator);
	
        // Initialization of the ned and nwu publishers
        // translator.ned_pub = nh.advertise<geometry_msgs::TransformStamped>("world/tf_ned", 1);
        // translator.nwu_pub = nh.advertise<geometry_msgs::TransformStamped>("world/tf_nwu", 1);
    *
    */

    ROS_INFO("Node started");

    ros::Rate rate_hz(1.0);  // rate of 1 sec to publish the messages
    while (nh.ok())
    {
        translator.ned_pub.publish(translator.ned);
        translator.nwu_pub.publish(translator.nwu);

        ros::spinOnce();
        rate_hz.sleep();
    }
    return 0;
};

