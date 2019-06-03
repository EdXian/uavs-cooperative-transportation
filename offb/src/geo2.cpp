#include <ros/ros.h>
#include <geometric_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <qptrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mav_msgs/Actuators.h>
#include <geometric_controller.h>



gazebo_msgs::ModelStates model_states;
geometry_msgs::PoseStamped payload_pose , follower_pose;
geometry_msgs::PoseStamped desired_pose;
geometry_msgs::TwistStamped follower_vel;
Eigen::Vector3d pose,vel,ang   ;
Eigen::Vector4d ori ;
Eigen::Matrix3d payload_Rotation;

unsigned int tick=0;
bool flag = false;
bool force_control = false;
gazebo_msgs::LinkStates linkstates;
Eigen::Matrix3d payload_link1_Rotation;

Eigen::Vector3d p_c2;
void model_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    model_states = *msg;
    for(unsigned int i=0;i<model_states.name.size();i++){
        if(model_states.name[i].compare("payload")==0){
            payload_pose.pose.position = model_states.pose[i].position;
            payload_pose.pose.orientation= model_states.pose[i].orientation;

            tf::Quaternion quat1(payload_pose.pose.orientation.x,
                                 payload_pose.pose.orientation.y,
                                 payload_pose.pose.orientation.z,
                                 payload_pose.pose.orientation.w);

//                tf::Matrix3x3(quat1).getRPY(payload_attitude(0), payload_attitude(1),  payload_attitude(2));
//                std::cout << " ===========payload_attitude ============="<<std::endl
//                          << payload_attitude <<std::endl;
            double x , y, z,w;
            x=payload_pose.pose.orientation.x;
            y=payload_pose.pose.orientation.y;
            z=payload_pose.pose.orientation.z;
            w=payload_pose.pose.orientation.w;

            payload_Rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;

            tf::Quaternion Q(
            payload_pose.pose.orientation.x,
            payload_pose.pose.orientation.x,
            payload_pose.pose.orientation.x,
            payload_pose.pose.orientation.w   );
//            tf::Matrix3x3(Q).getRPY(payload_roll,payload_pitch,payload_yaw);

        }
        if(model_states.name[i].compare("firefly1")==0){

        }
        if(model_states.name[i].compare("firefly2")==0){
            follower_pose.pose.position = model_states.pose[i].position;
            follower_pose.pose.orientation = model_states.pose[i].orientation;
            follower_vel.twist.linear = model_states.twist[i].linear;
            follower_vel.twist.angular = model_states.twist[i].angular;

            ori<< follower_pose.pose.orientation.w, follower_pose.pose.orientation.x,follower_pose.pose.orientation.y,follower_pose.pose.orientation.z;
            pose << follower_pose.pose.position.x , follower_pose.pose.position.y, follower_pose.pose.position.z;
            vel  << follower_vel.twist.linear.x, follower_vel.twist.linear.y, follower_vel.twist.linear.z;
            ang << follower_vel.twist.angular.x, follower_vel.twist.angular.y,follower_vel.twist.angular.z;
            tf::Quaternion Q(
            follower_pose.pose.orientation.x,
            follower_pose.pose.orientation.y,
            follower_pose.pose.orientation.z,
            follower_pose.pose.orientation.w   );

        }

    }
}

void link_cb(const gazebo_msgs::LinkStates::ConstPtr &msg){
 linkstates = *msg;
    if(linkstates.name.size()>0){
        for(unsigned int i=0 ;i<linkstates.name.size() ;  i++){

                if(linkstates.name[i].compare("payload_link1") == 0)
                {
                    double x , y, z, w;
                    x=linkstates.pose[i].orientation.x;
                    y=linkstates.pose[i].orientation.y;
                    z=linkstates.pose[i].orientation.z;
                    w=linkstates.pose[i].orientation.w;

                    payload_link1_Rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                                2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                                2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;

        }


                if(linkstates.name[i].compare("payload::payload_rec_g_box")==0){

                    p_c2 << linkstates.pose[i].position.x ,
                            linkstates.pose[i].position.y ,
                            linkstates.pose[i].position.z;

                }
        }
    }

}

geometry_msgs::WrenchStamped wrench1;
geometry_msgs::Point est_force;

geometry_msgs::WrenchStamped wrench2;

void force2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    //wrench2 = *msg;
    Eigen::Vector3d wrench,data;
    wrench << msg->wrench.force.x ,  msg->wrench.force.y , msg->wrench.force.z;
       data     = payload_link1_Rotation* wrench;

//    wrench2.wrench.force.x= data(0);
//    wrench2.wrench.force.y= data(1);
//    wrench2.wrench.force.z= data(2);

    wrench2.wrench.force.x= -1.0*est_force.x;
    wrench2.wrench.force.y= -1.0*est_force.y;
    //use
//    std::cout <<"====f====="<<std::endl;
//    std::cout <<    est_force.x  <<"  "<<est_force.y   <<std::endl;

    //    std::cout << "frame : " << msg->header.frame_id.c_str() << std::endl;
}
void force1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){

    Eigen::Vector3d wrench,data;
    wrench << msg->wrench.force.x ,  msg->wrench.force.y , msg->wrench.force.z;
       data             =payload_Rotation*wrench;

//    wrench1.wrench.force.x= data(0);
//    wrench1.wrench.force.y= data(1);
//    wrench1.wrench.force.z= data(2);
       wrench1.wrench.force.x = msg->wrench.force.x;
       wrench1.wrench.force.y = msg->wrench.force.y;
       wrench1.wrench.force.z = msg->wrench.force.z;

}
//mav_msgs::Actuators ;
void  est_force_cb(const geometry_msgs::Point::ConstPtr& msg){
    est_force = *msg;
}
geometry_msgs::Point trigger;
bool triggered;
trajectory_msgs::MultiDOFJointTrajectory cmd;
int main(int argc, char **argv)
{

    //rostopic
  ros::init(argc, argv, "geo2");
  ros::NodeHandle nh;
  ros::Subscriber model_sub20 = nh.subscribe<gazebo_msgs::ModelStates>
           ("/gazebo/model_states", 5, model_cb);

//   ros::Subscriber force2_sub = nh.subscribe<geometry_msgs::WrenchStamped>
//           ("/ft_sensor2_topic", 3, force2_cb);
   ros::Subscriber force1_sub = nh.subscribe<geometry_msgs::WrenchStamped>
           ("/ft_sensor1_topic", 3, force1_cb);

   ros::Subscriber est_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force_estimate", 3, est_force_cb);

   ros::Publisher  trigger_pub = nh.advertise<geometry_msgs::Point>("/follower_trigger", 2);
   //ros::Publisher   cmd_pub= nh.advertise<trajectory_msgs::MultiDOFJo
  // ros::Publisher  motors_pub= nh.advertise<mav_msgs::Actuators>intTrajectory>("/firefly2/command/trajectory", 2);
   ros::Publisher   traj_pub= nh.advertise<geometry_msgs::PoseStamped>("/firefly2/command/pose", 2);


   nh.setParam("/start2",false);
   nh.setParam("/force_control",false);

   triggered  = false;

   ros::Rate loop_rate(50.0);


    Eigen::Vector4d output;

    desired_pose.pose.position.x = -0.3;
    desired_pose.pose.position.y = 0.0;
    desired_pose.pose.position.z = 1.3;

       while(ros::ok()){
           if(nh.getParam("/start2",flag)){

           };
           if(nh.getParam("/force_control",force_control)){

           };

           double flx , fly ,ffx ,ffy;
           Eigen::Vector3d fs;
           flx = wrench1.wrench.force.x;
           fly = wrench1.wrench.force.y;
           ffx = -1.0*est_force.x;
           ffy = -1.0*est_force.y;


           double ft = sqrt( ffx * ffx + ffy *ffy );

           if((ft>0.3)){
               triggered = true;
           }else if((triggered)&&((ft<0.3)&&(ft>0.2))){
               triggered =true;
           }else {
                triggered= false;

            }

           trajectory_msgs::MultiDOFJointTrajectoryPoint   point;
            geometry_msgs::Twist  acc;
            geometry_msgs::Transform transform;
            geometry_msgs::PoseStamped force;

            Eigen::Vector4d output,force_cmd;

           if((triggered)&&(force_control)){

            Eigen::Vector3d p,v,a;
            p<<0,0,0;
            v<<0.0,0.0,0.0;
            a << (0-ffx)/1.5 , (0-ffy)/1.5 , 5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0-vel(2)) ;
            desired_pose.pose.position.x = pose(0);
            desired_pose.pose.position.y = pose(1);
            force.pose.position.x = a(0) + 1.0*(0-vel(0));
            force.pose.position.y = a(1) + 1.0*(0-vel(1));
            force.pose.position.z = a(2) + 0.5*0.5*9.8;
             trigger.x = 1;

           }else{

            Eigen::Vector3d p,v,a;
            v<<0.0,0.0,0.0;
            a<<0.0,0.0,0.0;
            p<< desired_pose.pose.position.x, desired_pose.pose.position.y,desired_pose.pose.position.z;
            force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0))+1*(0-vel(0));
            force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1))+1*(0-vel(1));
            force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2))+1*(0-vel(2))+ 0.5*0.5*9.8;
           // std::cout <<"position_control"<<std::endl;
            trigger.x = 0;
           }

           trigger.y  = ft;

           traj_pub.publish(force);
           trigger_pub.publish(trigger);


           ros::spinOnce();
           loop_rate.sleep();
       }


}
