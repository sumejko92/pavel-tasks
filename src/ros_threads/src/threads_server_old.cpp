#include "ros/ros.h"
#include "ros_threads/TimeDelay.h"
//#include "ctime"
#include "string"
#include "thread"
#include "chrono"

void start_thread(int td,std::string response)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(int(td*1000)));
	ROS_INFO_STREAM("Sending back response AFTER DELAY: " + response);
}


bool serviceCallback(ros_threads::TimeDelay::Request  &req,
         ros_threads::TimeDelay::Response &res)
{
  // grab the unix time
  //std::time_t result = std::time(nullptr);
  //res.Time = std::to_string(result);
  ros::Time result = ros::Time::now();
  int milisec = result.nsec/1000;
  res.Time = std::to_string(result.sec) + "." + std::to_string(milisec);
  
  ROS_INFO("Receiving request: Delay=%ld", (long int)req.Delay_s);

  if(req.Delay_s > 0 && req.Delay_s < 3)
  {
	std::thread t(start_thread,req.Delay_s,res.Time);
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
  ros::init(argc, argv, "threads_server_old");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/unix_time_now", serviceCallback);
  ROS_INFO("Feel free to call the service");

  ros::Rate rate(1.0);
  while (n.ok()){
	rate.sleep();
	ros::spinOnce();
	}
 
  return 0;
}

