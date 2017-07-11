#include "ros/ros.h"
#include "ros_threads/TimeDelay.h"
#include "random"
#include "string"
#include "thread"
#include "sstream"
#include <math.h> 

ros::ServiceClient client;

void start_thread()
{
    ros_threads::TimeDelay srv;

    double numeric;

    ros::Rate r(1.0);
    while(1)
    {
	    int random_num=rand()% 4 + 1;	
	    srv.request.Delay_s = random_num; 


	    if (client.call(srv))
	    {
		ros::Time current = ros::Time::now();
		//convert the string response to int
		std::stringstream(srv.response.Time)>>numeric;
		double res,unused;
		res = modf (numeric,&unused);
		int difference = current.nsec/1000000 - int(res*1000);
		ROS_INFO("Time Difference is : %d ms ",difference);
	    }
	    else
	    {
		ROS_ERROR("Failed to call service /unix_time_now");
	    }
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "threads_client_old");

  ros::NodeHandle n;
  client = n.serviceClient<ros_threads::TimeDelay>("/unix_time_now");

  std::thread t(start_thread);
  t.detach();

  ros::Rate rate(2.0);
  while (n.ok()){
	ros::Time time = ros::Time::now();

	ROS_INFO("Current time=%d.%d", time.sec,time.nsec);

	rate.sleep();
	ros::spinOnce();
	}
 

  return 0;
}
