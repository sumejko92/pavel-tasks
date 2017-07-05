#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>


tf::Transform transform;

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
    std::cout << "X:"<<std::endl;
    std::cout << vect.at(0)<<std::endl;
    std::cout << "Y:"<<std::endl;
    std::cout << vect.at(1)<<std::endl;
    std::cout << "Z:"<<std::endl;
    std::cout << vect.at(2)<<std::endl;
    std::cout << "ROLL:"<<std::endl;
    std::cout << vect.at(3)<<std::endl;
    std::cout << "PITCH:"<<std::endl;
    std::cout << vect.at(4)<<std::endl;
    std::cout << "YAW:"<<std::endl;
    std::cout << vect.at(5)<<std::endl;
    std::cout << " "<<std::endl; 

    //fill the msg
    transform.setOrigin( tf::Vector3(vect.at(0), vect.at(1), vect.at(2)) );
    transform.setRotation( tf::Quaternion(vect.at(3), vect.at(4), vect.at(5), 1) );
   
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher_base");
  ros::NodeHandle n;

  ROS_INFO("Node started");
 // ros::Publisher tf_pub = n.advertise<geometry_msgs::TransformStamped>("world/base_tf_enu", 10);

  tf::TransformBroadcaster br;
 // tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  
  std::string input_str;

  ros::Rate rate(10.0);
  while (n.ok()){

  	ROS_INFO("Awaiting user input of 6 numbers divided with ',': ");
  	std::cin >> input_str;
	process_input(input_str);
  	
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world/base_tf_enu"));
        //tf_pub.publish(transform);

        rate.sleep();
  }
  return 0;
};
