/*
 * Copyright (c) ??? - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Authors: Pavel Shumejko
 */

#include "ros/ros.h"
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>


/**
 * This class contains the functions for a ros node that subscribes to the image stream of your webcam and provides a service called /take_snapshot. This service will take the current image of the webcam and save it to file as JPG in /tmp/webcam/###.jpg, where ### is a number that is incremented every time the picture is taken.
 *
 */

class Webcam
{

private:
    //! @name global variables
    //@{
    sensor_msgs::Image g_img;    // Image variable
    //@}

public:
    Webcam() { }    

    /**
     * Callback function for the subscriber to /usb_cam/image_raw
     * @param msg gets the image
     */
    void streamCallback(const sensor_msgs::ImageConstPtr&);

    /**
     * This function is called when the service is called.
     * It takes a snapshot from the ongoing webcam stream and saves it as .jpg in /tmp/webcam/
     */
    bool takeSnapshot(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

};

void Webcam::streamCallback(const sensor_msgs::ImageConstPtr& msg)
{
    g_img = *msg;    //takes the image passed in the callback function into the global variable for global use
}

bool Webcam::takeSnapshot(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{
    ROS_INFO("Service called!");

    int img_count = 0;    // variable that stores the number of the image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(g_img, sensor_msgs::image_encodings::BGR8);

    std::stringstream img_name;    // variable for the filepath+name of the image

    while(1)
    {
        img_name<<"/tmp/webcam/ "<<img_count<<".jpg";    // forms the path

        std::ifstream ifile(img_name.str());    // checks if there is already a file with the same name

	// if no save the image in /temp/webcam/ as a .jpg 
        if (!ifile)    
        {   
            ROS_ASSERT(cv::imwrite(img_name.str(),cv_ptr->image));    // saves the image
            img_count = img_count+1;
	    break;
        }
        else
        {  
            img_count = img_count+1;	 
            img_name.str("");  
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    // rosnode inicialization
    ros::init(argc, argv, "webcam_subscriber");

    // create the handle
    ros::NodeHandle n;

    // Create the object
    Webcam webcam;

    ROS_INFO("Node Started");

    ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1,&Webcam::streamCallback,&webcam);
    ros::ServiceServer service_snapshot = n.advertiseService("/take_snapshot",
        &Webcam::takeSnapshot,&webcam);

    ros::spin();
    cv::destroyWindow("Image window");

    return 0;
}
