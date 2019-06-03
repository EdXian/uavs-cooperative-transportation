#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
std_msgs::Float64 data;
void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  data = *msg;

  std::cout<< data.data<<std::endl;

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "subtest");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/trigger/x", 10, chatterCallback);

  ros::spin();

  return 0;
}
