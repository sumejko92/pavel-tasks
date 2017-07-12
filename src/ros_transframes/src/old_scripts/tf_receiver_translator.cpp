#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher ned_pub;
ros::Publisher nwu_pub;
geometry_msgs::TransformStamped ned;
geometry_msgs::TransformStamped nwu;

void enuCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
  geometry_msgs::TransformStamped enu;

  //grab the enu msg
  enu=*msg;

  //std::cout << temp.transform.translation.x <<std::endl; 

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


int main(int argc, char** argv){

  ros::init(argc, argv, "tf_receiver_translator");

  ros::NodeHandle node;

  ros::Subscriber enu_sub = node.subscribe("world/base_tf_enu", 1, enuCallback);
  ned_pub = node.advertise<geometry_msgs::TransformStamped>("world/tf_ned", 1);
  nwu_pub = node.advertise<geometry_msgs::TransformStamped>("world/tf_nwu", 1);

  tf::TransformListener listener;

  ros::Rate rate(1.0);
  while (node.ok()){
/*
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("world", "world/base_tf_enu",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

   //get transform values
   double x = transform.getOrigin().x();
   double y = transform.getOrigin().y();
   double z = transform.getOrigin().z();
   double rx = transform.getRotation().x();
   double ry = transform.getRotation().y();
   double rz = transform.getRotation().z();
*/  
   ned_pub.publish(ned);
   nwu_pub.publish(nwu);

   ros::spinOnce();
   rate.sleep();
  }
  return 0;
};

