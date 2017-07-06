#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <geometry_msgs/TransformStamped.h>

//tf::Transform transform;
geometry_msgs::TransformStamped tfs;

//receives the input string and extracts the num values values 
void process_input(const std::string& input)
{
    ROS_INFO("You entered:");
    std::cout << input << std::endl;

    std::vector<int> vect;
    std::stringstream ss(input);

    int i;

    while (ss >> i)
    {
        vect.push_back(i);

        if (ss.peek() == ',')
            ss.ignore();
    }
    
    // print the parsed values
    std::cout << " "<<std::endl;
    ROS_INFO("Parsed:");
    std::cout << "X:";
    std::cout << vect.at(0)<<std::endl;
    std::cout << "Y:";
    std::cout << vect.at(1)<<std::endl;
    std::cout << "Z:";
    std::cout << vect.at(2)<<std::endl;
    std::cout << "ROLL:";
    std::cout << vect.at(3)<<std::endl;
    std::cout << "PITCH:";
    std::cout << vect.at(4)<<std::endl;
    std::cout << "YAW:";
    std::cout << vect.at(5)<<std::endl;
    std::cout << " "<<std::endl; 

    // fill the msg
    tfs.transform.translation.x = vect.at(0);
    tfs.transform.translation.y = vect.at(1);
    tfs.transform.translation.z = vect.at(2);
    tfs.transform.rotation.x = vect.at(3);
    tfs.transform.rotation.y = vect.at(4);
    tfs.transform.rotation.z = vect.at(5);
    tfs.transform.rotation.w = 1;
   
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher_base");
  ros::NodeHandle n;

  ROS_INFO("Node started");
  ros::Publisher tf_pub = n.advertise<geometry_msgs::TransformStamped>("world/base_tf_enu", 1);

  tf::TransformBroadcaster br;


  tfs.header.stamp = ros::Time::now();
  tfs.header.frame_id = "world";
  tfs.child_frame_id = "world/base_tf_enu";

  tfs.transform.translation.x = 0.0;
  tfs.transform.translation.y = 0.0;
  tfs.transform.translation.z = 0.0;
  tfs.transform.rotation.x = 0.0;
  tfs.transform.rotation.y = 0.0;
  tfs.transform.rotation.z = 0.0;
  tfs.transform.rotation.w = 1.0;
  
  std::string input_str;

  ros::Rate rate(1.0);
  while (n.ok()){

  	ROS_INFO("Awaiting user input of 6 numbers divided with ',': ");
  	std::cin >> input_str;
	process_input(input_str);
  	

	//publishes ONLY ONCE per cycle due to waiting for raw input, there must be an easy fix to publish asinc but multithreading would be an overkill
        tf_pub.publish(tfs);


	//broadcasts the transformation every second
	br.sendTransform(tfs);
        
        rate.sleep();
  }
  return 0;
};
