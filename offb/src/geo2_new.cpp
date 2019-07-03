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
#define length   0.18;


gazebo_msgs::ModelStates model_states;
geometry_msgs::PoseStamped payload_pose , follower_pose;
geometry_msgs::PoseStamped desired_pose;
geometry_msgs::TwistStamped follower_vel;
Eigen::Vector3d pose,vel,ang   ;
Eigen::Vector4d ori ;


unsigned int tick=0;
bool flag = false;
bool force_control = false;
gazebo_msgs::LinkStates linkstates;
Eigen::Matrix3d payload_link1_Rotation;

Eigen::Vector3d p_c2;
Eigen::Vector3d last_p_c2;
Eigen::Vector3d v_c2;
Eigen::Matrix3d payload_Rotation;
Eigen::Matrix3d uav_rotation;

geometry_msgs::WrenchStamped wrench1;
geometry_msgs::Point est_force;

geometry_msgs::WrenchStamped wrench2;

geometry_msgs::Point  data;

Eigen::Vector3d  v_c2_truth, v_c2_b_truth;
Eigen::Vector3d  p_c2_truth;
Eigen::Matrix3d  payload_rotatiob_truth;
Eigen::Vector3d  v_c2_y;
void model_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){

    gazebo_msgs::LinkStates links = *msg;
    if(links.name.size()>0){
        for(unsigned int i=0; i<links.name.size(); i++){
            if (links.name[i].compare("payload::payload_rec_g_box") == 0 ) {
                Eigen::Vector3d vec;
                v_c2_truth << links.twist[i].linear.x, links.twist[i].linear.y, links.twist[i].linear.z;
                p_c2_truth <<links.pose[i].position.x, links.pose[i].position.y, links.pose[i].position.z;
                vec<<  0 ,links.twist[i].linear.y, 0;

                double w,x,y,z;
                x=links.pose[i].orientation.x;
                y=links.pose[i].orientation.y;
                z=links.pose[i].orientation.z;
                w=links.pose[i].orientation.w;
                tf::Quaternion q(x,y,z,w);
                double roll,pitch,yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                //body to inertial frame
                payload_rotatiob_truth<< cos(yaw), -sin(yaw) , 0,
                                         sin(yaw),  cos(yaw),  0,
                                         0   ,   0       ,   1;

                v_c2_y = payload_rotatiob_truth * vec ;

                v_c2_b_truth =  payload_Rotation.transpose()* v_c2_truth;

            }
        }

    }
}

Eigen::Vector3d pfc;
 Eigen::Vector3d f, fb ,vb;
void  est_force_cb(const geometry_msgs::Point::ConstPtr& msg){
    est_force = *msg;

    f<< -est_force.x, -est_force.y, est_force.z;

    pfc  =    pose    -  uav_rotation * Eigen::Vector3d(0,0,0.05);// offset x from uav to connector

    p_c2 = pfc +  (f/f.norm())* 0.18 ;

    v_c2 =   (p_c2 - last_p_c2)/0.02;
    double payload_yaw = atan( v_c2(1) / v_c2(0));

    //from inertial frame to body frame
    std::cout <<    "v_c2 error : "<<(v_c2).transpose() << std::endl;


    data.x = (p_c2 - p_c2_truth)(0);
    data.y = (p_c2 - p_c2_truth)(1);
    data.z = (p_c2 - p_c2_truth)(2);
    payload_yaw = atan( v_c2_truth(1) / v_c2_truth(0));
    //body frame to inertial frame.
    payload_Rotation << cos(payload_yaw), -sin(payload_yaw) ,0,
                        sin(payload_yaw), cos(payload_yaw) ,0,
                            0           ,      0           ,1;



    //change the force from inertial frame to body.


     last_p_c2 = p_c2;

}


nav_msgs::Odometry odom;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
    ori<< odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z;
    pose << odom.pose.pose.position.x , odom.pose.pose.position.y, odom.pose.pose.position.z;
    vel  << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
    ang << odom.twist.twist.angular.x, odom.twist.twist.angular.y,odom.twist.twist.angular.z;

    double w,x,y,z;
    x=msg->pose.pose.orientation.x;
    y=msg->pose.pose.orientation.y;
    z=msg->pose.pose.orientation.z;
    w=msg->pose.pose.orientation.w;

    uav_rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                   2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                   2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;
}




geometry_msgs::Point trigger;
bool triggered;
trajectory_msgs::MultiDOFJointTrajectory cmd;
int main(int argc, char **argv)
{

    //rostopic
  ros::init(argc, argv, "geo2");
  ros::NodeHandle nh;

  ros::Subscriber est_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force_estimate", 3, est_force_cb);

  ros::Publisher  trigger_pub = nh.advertise<geometry_msgs::Point>("/follower_trigger", 2);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
           ("/firefly2/odometry_sensor1/odometry", 3, odom_cb);
  ros::Subscriber  model_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",1,model_cb);

  ros::Publisher   traj_pub= nh.advertise<geometry_msgs::PoseStamped>("/firefly2/command/pose", 2);
  ros::Publisher   vc2_pub = nh.advertise<geometry_msgs::Point>("vc2_error",1);

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

            Eigen::Vector3d p,v,a, ab;
            p<<0,0,0;
            v<<0.0,0.0,0.0;


            fb = payload_rotatiob_truth *f;
            vb = payload_rotatiob_truth *vel;
               // v_c2_b_truth


//            ab <<  ( 0-fb(0))*0.2  + ( v_c2_b_truth(0)),
//                    fb(1) + 0.1*(0 - v_c2_b_truth(1)) ,
//                    5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0-vel(2)) + 0.5*0.5*9.8;



            // + 0.1*( vb(0))

//            ab <<   (0-fb(0))*0.2 + vb(0),
//                    fb(1) + 0.1*(0-vb(1)),
//                    5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0-vel(2)) + 0.5*0.5*9.8;

          //  a = payload_rotatiob_truth.transpose() * ab;


            a << 1.0*(0-vel(0)) + (0-ffx)/1.5 + 0.3*v_c2_y(0),
                 1.0*(0-vel(1)) + (0-ffy)/1.5 + 0.3*v_c2_y(1),
                 5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0-vel(2)) + 0.5*0.5*9.8;



            //a << (0-ffx)/1.5, (0-ffy)/1.5 , 5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0-vel(2)) ;


            desired_pose.pose.position.x = pose(0);
            desired_pose.pose.position.y = pose(1);

            force.pose.position.x = a(0);           //a(0) ;
            force.pose.position.y = a(1);
            force.pose.position.z = a(2) ;


            trigger.x = 1;

           }else{

            Eigen::Vector3d p,v,a;
            v<<0.0,0.0,0.0;
            a<<0.0,0.0,0.0;
            p<< desired_pose.pose.position.x, desired_pose.pose.position.y,desired_pose.pose.position.z;
            force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0))+1*(0-vel(0));
            force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1))+1*(0-vel(1));
            force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2))+1*(0-vel(2))+ 0.5*0.5*9.8;
            trigger.x = 0;

           }

           trigger.y  = ft;

           traj_pub.publish(force);
           trigger_pub.publish(trigger);



           vc2_pub.publish(data);


           ros::spinOnce();
           loop_rate.sleep();
       }


}
