#include <ros/ros.h>
//#include <plot_rviz.h>
#include "plot_rviz.h"

rvizplot::rvizplot(){

  pose_pub = nh.advertise<visualization_msgs::MarkerArray>("/rviz_pose",1);
  desired_pose_pub = nh.advertise<visualization_msgs::MarkerArray>("/rviz_desired_pose",1);
  pose_sub = nh.subscribe("/gazebo/model_states",1,&rvizplot::pose_cb,this);
  desired_pose_sub = nh.subscribe("/drone1/desired_position",1,&rvizplot::desired_pose_cb,this);
  count =0;
}

void rvizplot::desired_pose_cb(const geometry_msgs::Point::ConstPtr& msg){

  visualization_msgs::Marker marker;
  marker.ns = "basic_shapes";

  marker.header.frame_id = "path";
  marker.header.stamp = ros::Time::now();
  marker.id =count ;
  count++;


  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  marker.color.g = 0.0;
  marker.color.r = 0.0;

  marker.lifetime = ros::Duration();

  marker.pose.position.x = msg->x;
  marker.pose.position.y = msg->y;
  marker.pose.position.z = 0.40;

  if(count % 10==0){
   desired_pose_marker.markers.push_back(marker);
     desired_pose_pub.publish(desired_pose_marker);
  }

}

void rvizplot::pose_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
  visualization_msgs::Marker marker;
  marker.ns = "basic_shapes";

  marker.header.frame_id = "path";
  marker.header.stamp = ros::Time::now();
  marker.id =count2;
  count2++;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 1.0;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;
  marker.color.a = 0.5;
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.color.r = 1.0;
//    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.mesh_resource = "package://rotors_description/meshes/firefly.dae";
  marker.lifetime = ros::Duration();

  gazebo_msgs::ModelStates model=*msg;
  for(unsigned int i=0;i<model.pose.size();i++ ){
    if(model.name[i].compare("payload") == 0 ){

//      double x, y, z, w;
//      x= model.pose[i].orientation.x;
//      y= model.pose[i].orientation.y;
//      z= model.pose[i].orientation.z;
//      w= model.pose[i].orientation.w;

//      tf::Quaternion q(x,y,z,w);

//      tf::Matrix3x3(q).getRPY();
//      marker.pose.position.x = msg->x;
//      marker.pose.position.y = msg->y;
//      marker.pose.position.z = 0.40;
      marker.pose.position = model.pose[i].position;
      marker.pose.position.z = 0.40;
      marker.pose.orientation = model.pose[i].orientation;
      if(count2 % 200==0){
        pose_marker.markers.push_back(marker);
        pose_pub.publish(pose_marker);
      }

    }
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot_rviz");
  rvizplot plot;
  ros::spin();

}
