#include "ros/ros.h"
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

//global variables
sensor_msgs::Image img_;
static const std::string OPENCV_WINDOW = "Image window";
int img_count = 0;

void stream_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  //ROS_INFO("We have a stream");

  img_ = *msg;

}

bool take_snapshot(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{
  ROS_INFO("Service called!");

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img_, sensor_msgs::image_encodings::BGR8);
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);

  std::stringstream sstream;
  sstream<<"/tmp/webcam/ "<<img_count<<".jpg";

  ROS_ASSERT(cv::imwrite(sstream.str(),cv_ptr->image));
  img_count=img_count+1;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "webcam_subscriber");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1, stream_Callback);

  ros::ServiceServer service_snapshot = n.advertiseService("/take_snapshot", take_snapshot);

  ros::spin();
  cv::destroyWindow("Image window");
  return 0;
}