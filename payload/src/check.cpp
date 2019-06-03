#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "lpf.h"
#include "sensor_msgs/Imu.h"




geometry_msgs::TwistStamped vel,vel2;
geometry_msgs::TwistStamped set_vel_,set_vel2_;
geometry_msgs::PoseStamped pos,pos2;
geometry_msgs::PoseStamped mocap_pos,mocap_pos2;
sensor_msgs::Imu imu2;
geometry_msgs::Point mvel,mvel2;
geometry_msgs::Point mpos,mpos2;
geometry_msgs::Point mmocap_pos,mmocap_pos2;
geometry_msgs::Point mset_vel,mset_vel2;
geometry_msgs::Point imu2_point;

lpf lpfx(2,0.02);
lpf lpfy(2,0.02);
lpf lpfz(2,0.02);

void vel_cb(const  geometry_msgs::TwistStamped::ConstPtr &msg){
  vel = *msg;
  mvel.x =   lpfx.filter(vel.twist.linear.x);
  mvel.y =   lpfy.filter(vel.twist.linear.y);
  mvel.z =   lpfz.filter(vel.twist.linear.z);



}
void pos_cb(const  geometry_msgs::PoseStamped::ConstPtr &msg){
  pos =*msg;

  mpos.x = pos.pose.position.x;
  mpos.y = pos.pose.position.y;
  mpos.z = pos.pose.position.z;

}
void mocap_cb(const  geometry_msgs::PoseStamped::ConstPtr &msg){
  mocap_pos = *msg;
  mmocap_pos.x = mocap_pos.pose.position.x;
  mmocap_pos.y = mocap_pos.pose.position.y;
  mmocap_pos.z = mocap_pos.pose.position.z;
}

void vel2_cb(const  geometry_msgs::TwistStamped::ConstPtr &msg){
  vel2 = *msg;
  mvel2.x = vel2.twist.linear.x;
  mvel2.y = vel2.twist.linear.y;
  mvel2.z = vel2.twist.linear.z;
}
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu2 = *msg;
  imu2_point.x = imu2.linear_acceleration.x;
  imu2_point.y = imu2.linear_acceleration.y;
  imu2_point.z = imu2.linear_acceleration.z;
}
void pos2_cb(const  geometry_msgs::PoseStamped::ConstPtr &msg){
  pos2 =*msg;
  mpos2.x = pos2.pose.position.x;
  mpos2.y = pos2.pose.position.y;
  mpos2.z = pos2.pose.position.z;

}
void mocap2_cb(const  geometry_msgs::PoseStamped::ConstPtr &msg){
  mocap_pos2 = *msg;
  mmocap_pos2.x = mocap_pos2.pose.position.x;
  mmocap_pos2.y = mocap_pos2.pose.position.y;
  mmocap_pos2.z = mocap_pos2.pose.position.z;
}

void set_vel_cb(const geometry_msgs::TwistStamped::ConstPtr & msg){
   set_vel_ = *msg;
  mset_vel.x = set_vel_.twist.linear.x;
  mset_vel.y = set_vel_.twist.linear.y;
  mset_vel.z = set_vel_.twist.linear.z;
}
void set_vel2_cb(const geometry_msgs::TwistStamped::ConstPtr & msg){
  set_vel2_ = *msg;
 mset_vel2.x = set_vel2_.twist.linear.x;
 mset_vel2.y = set_vel2_.twist.linear.y;
 mset_vel2.z = set_vel2_.twist.linear.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "check");
  ros::NodeHandle nh;

  ros::Subscriber vel_sub = nh.subscribe("/drone2/mavros/local_position/velocity", 10, vel_cb);
  ros::Subscriber pos_sub = nh.subscribe("/drone2/mavros/local_position/pose", 10, pos_cb);
  ros::Subscriber mocap_sub = nh.subscribe("/vrpn_client_node/RigidBody2/pose",10,mocap_cb);
  ros::Subscriber set_vel_sub = nh.subscribe("/drone2/mavros/setpoint_velocity/cmd_vel",10,set_vel_cb);
  ros::Subscriber imu2_sub = nh.subscribe("/imu1/mavros/imu/data_raw",10,imu_cb);
  ros::Publisher imu2_pub = nh.advertise<geometry_msgs::Point>("imu2",10);



  ros::Subscriber vel2_sub = nh.subscribe("/drone3/mavros/local_position/velocity", 10, vel2_cb);
  ros::Subscriber pos2_sub = nh.subscribe("/drone3/mavros/local_position/pose", 10, pos2_cb);
  ros::Subscriber mocap2_sub = nh.subscribe("/vrpn_client_node/RigidBody3/pose",10,mocap2_cb);
  ros::Subscriber set_vel2_sub = nh.subscribe("/drone3/mavros/setpoint_velocity/cmd_vel",10,set_vel2_cb);




  ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("pos",10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("vel",10);
  ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::Point>("mocap_pos",10);
  ros::Publisher pos2_pub = nh.advertise<geometry_msgs::Point>("pos2",10);
  ros::Publisher vel2_pub = nh.advertise<geometry_msgs::Point>("vel2",10);
  ros::Publisher mocap_pos2_pub = nh.advertise<geometry_msgs::Point>("mocap_pos2",10);

  ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::Point>("set_vel",10);

  ros::Publisher set_vel2_pub = nh.advertise<geometry_msgs::Point>("set_vel2",10);





  ros::Rate loop_rate(50);
  while(ros::ok()){




    pos_pub.publish(mpos);
    vel_pub.publish(mvel);
    mocap_pos_pub.publish(mmocap_pos);
    set_vel_pub.publish(mset_vel);


    pos2_pub.publish(mpos2);
    vel2_pub.publish( mvel2);
    mocap_pos2_pub.publish(mmocap_pos2);
    set_vel2_pub.publish(mset_vel2);
    imu2_pub.publish(imu2_point);


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
