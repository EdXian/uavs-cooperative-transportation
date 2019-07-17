#ifndef PLOT_RVIZ_H
#define PLOT_RVIZ_H
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
class rvizplot
{
public:
  rvizplot();
  void desired_pose_cb(const geometry_msgs::Point::ConstPtr& msg);
  void pose_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);


private:
  unsigned int count;
  unsigned int count2;

  ros::NodeHandle nh;
  ros::Publisher pose_pub;
  ros::Publisher desired_pose_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber desired_pose_sub;

  visualization_msgs::MarkerArray pose_marker;
  visualization_msgs::MarkerArray desired_pose_marker;


};

















#endif // PLOT_RVIZ_H
