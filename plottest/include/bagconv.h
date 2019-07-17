
#ifndef BAGCONV_H
#define BAGCONV_H
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
class bagconv
{
public:
  bagconv();

  void model_cb(const  gazebo_msgs::ModelStates::ConstPtr& msg  );
  void link_cb(const  gazebo_msgs::LinkStates::ConstPtr& msg  );
  void force1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void force2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void vel_est_cb(const  geometry_msgs::Point::ConstPtr& msg);

// others

  void traj_cb(const  geometry_msgs::Point::ConstPtr& msg  );
  void desired_traj_cb(const geometry_msgs::Point::ConstPtr& msg);
  void point_cb(const geometry_msgs::Point::ConstPtr& msg);
  void estimate_force_cb(const geometry_msgs::Point::ConstPtr& msg);
  void desired_force_cb(const geometry_msgs::Point::ConstPtr& msg);
  void trigger_cb(const geometry_msgs::Point::ConstPtr& msg);
  void desired_velocity_cb(const geometry_msgs::Point::ConstPtr& msg);
  void vel_est_b_cb(const geometry_msgs::Point::ConstPtr& msg);




private:

  ros::NodeHandle nh;
  ros::Subscriber model_sub;
  ros::Subscriber link_sub;
  ros::Subscriber force1_sub;
  ros::Subscriber force2_sub;



  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix3d payload_link1_rotation;
  Eigen::Matrix3d payload_link2_rotation;


  gazebo_msgs::ModelStates model_state;
  gazebo_msgs::LinkStates link_state;

  geometry_msgs::Point payload_vel;
  geometry_msgs::Point payload_pose;
  geometry_msgs::Point c2_vel_b;
  geometry_msgs::Point c2_vel;
  geometry_msgs::Point point;


//  traj_cb
//  desired_traj_cb
//  point_cb
//  estimate_force_cb
//  desired_force_cb
//  trigger_cb
//  desired_velocity_cb
//  vel_est_cb
  //vel_est_b_cb
   // ros::Publisher
};

#endif // BAGCONV_H
