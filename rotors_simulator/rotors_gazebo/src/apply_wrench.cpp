#include <ros/ros.h>
#include "std_msgs/String.h"

//#include "rotors_control/parameters_ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/BodyRequest.h>

double factor_2 = 1.0;
double factor_1 = 1.0;
geometry_msgs::PoseStamped payload_pose;
geometry_msgs::PoseStamped desired_pose;
Eigen::Matrix3d R_payload;

Eigen::Vector3d iris_1_pose;
Eigen::Vector3d iris_2_pose;
//Eigen::Vector3d payload_pose;
Eigen::Vector3d con_1_pose;
Eigen::Vector3d con_2_pose;
Eigen::Vector3d con_1_vel;
Eigen::Vector3d con_2_vel;

Eigen::Vector3d iris_1_vel;
Eigen::Vector3d iris_2_vel;
Eigen::Vector3d payload_vel;

double k=7, c=0.7;
double length=0.1;

Eigen::Vector3d  p1 ,p2;
Eigen::Vector3d  f1 , f2;
Eigen::Vector3d  last_f1 , last_f2;
Eigen::Vector3d  dv1 , dv2;

Eigen::Matrix3d  spring;
Eigen::Matrix3d  damp;
gazebo_msgs::ModelStates model_states;
gazebo_msgs::ApplyBodyWrench apply_wrench_iris2 , apply_wrench_iris1,apply_wrench_payload1,apply_wrench_payload2;

geometry_msgs::Point  point;
geometry_msgs::Point  point2;
geometry_msgs::Point  point3;

geometry_msgs::Point point4;

geometry_msgs::Point con1_vel;
geometry_msgs::Point con2_vel;
gazebo_msgs::BodyRequest body_name;



gazebo_msgs::LinkStates link_states;
void link_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
   link_states = *msg;
   if(link_states.name.size()>0){
       for (unsigned int i=0;i<link_states.name.size();i++) {
           if(link_states.name[i].compare("payload::payload_link1_box")==0){
                con_1_pose<< link_states.pose[i].position.x,
                            link_states.pose[i].position.y,
                            link_states.pose[i].position.z;
                con_1_vel<< link_states.twist[i].linear.x,
                            link_states.twist[i].linear.y,
                            link_states.twist[i].linear.z;
                con1_vel.x = link_states.twist[i].linear.x;
                con1_vel.y = link_states.twist[i].linear.y;
                con1_vel.z = link_states.twist[i].linear.z;

           }
           if(link_states.name[i].compare("payload::payload_link1_box")==0){
               con_2_pose<< link_states.pose[i].position.x,
                           link_states.pose[i].position.y,
                           link_states.pose[i].position.z;
               con_2_vel<< link_states.twist[i].linear.x,
                           link_states.twist[i].linear.y,
                           link_states.twist[i].linear.z;
               con2_vel.x = link_states.pose[i].position.x;
               con2_vel.y = link_states.pose[i].position.y;
               con2_vel.z = link_states.pose[i].position.z;
           }

       }
   }

}

void state_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
model_states = *msg;
if(model_states.name.size()>0){
    for(unsigned int i=0; i<model_states.name.size();i++){
        if(model_states.name[i].compare("firefly1")==0){
           iris_1_pose<< model_states.pose[i].position.x,
                         model_states.pose[i].position.y,
                         model_states.pose[i].position.z;
           iris_1_vel <<model_states.twist[i].linear.x,
                        model_states.twist[i].linear.y,
                        model_states.twist[i].linear.z;

        }
        if(model_states.name[i].compare("firefly2")==0){
            iris_2_pose<< model_states.pose[i].position.x,
                          model_states.pose[i].position.y,
                          model_states.pose[i].position.z;
            iris_2_vel <<model_states.twist[i].linear.x,
                         model_states.twist[i].linear.y,
                         model_states.twist[i].linear.z;
        }
        if(model_states.name[i].compare("payload")==0){
            payload_pose.pose.position = model_states.pose[i].position;
            payload_pose.pose.orientation = model_states.pose[i].orientation;
        }
    }

    double x, y, z, w;
    x = payload_pose.pose.orientation.x;
    y = payload_pose.pose.orientation.y;
    z = payload_pose.pose.orientation.z;
    w = payload_pose.pose.orientation.w;

    R_payload<<  w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                  2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                  2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


    damp<<0.2 ,  0   ,  0,
          0   ,  0.2 ,  0,
           0  ,  0   , 0.7;

    spring<< 5 ,  0   ,  0,
             0   , 5,  0,
             0  ,  0   , 5;

   p1=iris_1_pose-con_1_pose;
   p2=iris_2_pose-con_2_pose;

   dv1=iris_1_vel - con_1_vel;
   dv2=iris_2_vel-  con_2_vel;

    double delta1 = p1.norm() - length  ;
    double delta2 = p2.norm() - length   ;

    if(delta1>0){
        f1 =    spring *delta1*  p1/p1.norm()   +  damp* dv1/dv1.norm() ;

    }else{
        f1 <<0,0,0;

    }
    if(delta2>0){
        f2 =   spring *delta2*  p2/p2.norm()   +   damp* dv2/dv2.norm() ;
    }else{
        f2 <<0,0,0;
    }

    last_f1 = f1;
    last_f2  =f2;


    std::cout <<"+++++apply force+++++++++++++" <<std::endl;
    std::cout <<"f1" <<std::endl<<f1.transpose()<<std::endl;
    std::cout <<"f2" <<std::endl<<f2.transpose()<<std::endl;

    apply_wrench_iris1.request.wrench.force.x = -1*f1(0)  *factor_1;
    apply_wrench_iris1.request.wrench.force.y = -1*f1(1) *factor_1;
    apply_wrench_iris1.request.wrench.force.z = -1*f1(2)*factor_1;


    apply_wrench_iris2.request.wrench.force.x = -1*f2(0)*factor_1;
    apply_wrench_iris2.request.wrench.force.y =-1*f2(1) *factor_1;
    apply_wrench_iris2.request.wrench.force.z = -1*f2(2)*factor_1;



    apply_wrench_payload1.request.wrench.force.x = f1(0)  *factor_2 ;
    apply_wrench_payload1.request.wrench.force.y =f1(1) *factor_2 ;
    apply_wrench_payload1.request.wrench.force.z = f1(2)*factor_2 ;

    apply_wrench_payload2.request.wrench.force.x = f2(0)*factor_2 ;
    apply_wrench_payload2.request.wrench.force.y =f2(1)*factor_2 ;
    apply_wrench_payload2.request.wrench.force.z = f2(2)*factor_2;

    point4.x = f1(0);

    point4.y = f1(1);
    point4.z = f1(2);


}
//compute force
}




//the force sensor is in body frame (payload). Needing conversion before using
geometry_msgs::WrenchStamped wrench2;
void force2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    wrench2= *msg;
    Eigen::Vector3d tmp;
    Eigen::Vector3d data;
    tmp<< wrench2.wrench.force.x,
            wrench2.wrench.force.y,
            wrench2.wrench.force.z;
    data= R_payload*tmp;
    point2.x = data(0);
    point2.y = data(1);
    point2.z = data(2);

}
geometry_msgs::WrenchStamped wrench1;
void force1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    wrench1= *msg;
    Eigen::Vector3d tmp;
    Eigen::Vector3d data;
    tmp<< wrench1.wrench.force.x,
            wrench1.wrench.force.y,
            wrench1.wrench.force.z;
    data=  R_payload*tmp;
    point.x = data(0);
    point.y = data(1);
    point.z = data(2);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotors");
    ros::NodeHandle nh;
    double count =0;
  ros::ServiceClient wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>
                                                       ("/gazebo/apply_body_wrench");

  ros::ServiceClient dewrench_client = nh.serviceClient<gazebo_msgs::BodyRequest>
                                                       ("clear_body_wrenches");


  ros::Subscriber state_sub = nh.subscribe<gazebo_msgs::ModelStates>
          ("/gazebo/model_states", 3, state_cb);
  ros::Subscriber link_sub = nh.subscribe<gazebo_msgs::LinkStates>
          ("/gazebo/link_states", 3, link_cb);
  ros::Subscriber force1_sub = nh.subscribe<geometry_msgs::WrenchStamped>
          ("/ft_sensor1_topic", 3, force1_cb);
  ros::Subscriber force2_sub = nh.subscribe<geometry_msgs::WrenchStamped>
          ("/ft_sensor2_topic", 3, force2_cb);
  ros::Rate loop_rate(30);
  ros::Publisher force1_pub=nh.advertise<geometry_msgs::Point>("force1",5);
  ros::Publisher force4_pub=nh.advertise<geometry_msgs::Point>("forcepull",5);

  ros::Publisher con1vel=nh.advertise<geometry_msgs::Point>("con1_vel",5);
  ros::Publisher con2vel=nh.advertise<geometry_msgs::Point>("con2_vel",5);

  ros::Publisher force2_pub=nh.advertise<geometry_msgs::Point>("force2",5);

  body_name.request.body_name="payload";
  ros::Duration duration_(0.3) ;



    apply_wrench_iris1.request.body_name="firefly1::firefly1/base_link";
    apply_wrench_payload1.request.body_name="payload::payload_link2_box";
    apply_wrench_payload2.request.body_name="payload::payload_link1_box";
    apply_wrench_iris2.request.body_name="firefly2::firefly2/base_link";


    apply_wrench_payload1.request.duration = duration_;
    apply_wrench_payload2.request.duration = duration_;

    apply_wrench_iris1.request.reference_point= point;
    apply_wrench_iris1.request.duration = duration_;

    apply_wrench_iris2.request.reference_point= point;
    apply_wrench_iris2.request.duration = duration_;

    std::cout << "ok" <<std::endl;


    for(unsigned int i=0;i<200;i++){
        ros::spinOnce();
        loop_rate.sleep();
    }
 while (ros::ok())
  {


    dewrench_client.call(body_name);
    //dewrench_client.call();
    wrench_client.call(apply_wrench_iris1);
    wrench_client.call(apply_wrench_iris2);
    wrench_client.call(apply_wrench_payload1);
    wrench_client.call(apply_wrench_payload2);

    con1vel.publish(con1_vel);
    con2vel.publish(con2_vel);

    force1_pub.publish(point);
    force2_pub.publish(point2);
    force4_pub.publish(point4);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
