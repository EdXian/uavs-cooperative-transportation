#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "UKF/output.h"
#include "geometry_msgs/Point.h"
#include "kalmanfilter.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "lpf.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

#define l 0.75


sensor_msgs::Imu imu1_data;
sensor_msgs::Imu imu2_data;
geometry_msgs::PoseStamped marker1_pose;
geometry_msgs::PoseStamped marker2_pose;
geometry_msgs::Point point;
Eigen::Matrix3d Q_B1, Q_B2;
Eigen::Vector3d w_B1, w_B2;
Eigen::Vector3d a_B1, a_B2;
Eigen::Vector3d alpha;
Eigen::Vector3d w_I1, w_I2;
Eigen::Vector3d last_w_I1;
Eigen::Vector3d a_I1, a_I2;
Eigen::Vector3d r_p;
Eigen::Vector3d a_p;
//geometry_msgs::PointStamped point;
UKF::output Follower_Force ;
geometry_msgs::Point leader_force;
double lastime ;
double dt;
double dx ,dy ,dz;
void force_cb(const UKF::output::ConstPtr& msg){
  Follower_Force = *msg;

}



void imu1_cb(const sensor_msgs::Imu::ConstPtr& msg){

   //dt=ros::Time::now().toSec() - lastime;
  dt = 0.02;
  lastime = ros::Time::now().toSec();


        imu1_data = *msg;
        a_B1<<imu1_data.linear_acceleration.x
            ,imu1_data.linear_acceleration.y
            ,imu1_data.linear_acceleration.z;


        w_B1<<imu1_data.angular_velocity.x
            ,imu1_data.angular_velocity.y
            ,imu1_data.angular_velocity.z;


        a_I1 = Q_B1 * a_B1;

//        std::cout <<"a_I" <<std::endl<<a_I1
//                 <<std::endl;
//        std::cout <<"alpha" <<std::endl<<a_I1/5
//                 <<std::endl;

        a_I1[2] =  a_I1[2]-9.81;

        w_I1 = Q_B1 * w_B1;

        alpha = (w_I1 -  last_w_I1)/dt;

        last_w_I1 = w_I1;

}
void imu2_cb(const sensor_msgs::Imu::ConstPtr& msg){
        imu2_data = *msg;
        a_B2<<imu2_data.linear_acceleration.x
            ,imu2_data.linear_acceleration.y
            ,imu2_data.linear_acceleration.z;

        w_B2<<imu2_data.angular_velocity.x
            ,imu2_data.angular_velocity.y
            ,imu2_data.angular_velocity.z;

        a_I2 = Q_B2 * a_B2;
        a_I2[2] =  a_I2[2]-9.81;
        w_I2 = Q_B2 * w_B2;






}

double mocap1_roll , mocap1_yaw ,mocap1_pitch;
void mocap1_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
 marker1_pose =*msg;
 double x = marker1_pose.pose.orientation.x;
 double y = marker1_pose.pose.orientation.y;
 double z = marker1_pose.pose.orientation.z;
 double w = marker1_pose.pose.orientation.w;

 tf::Quaternion quat1(marker1_pose.pose.orientation.x,
                      marker1_pose.pose.orientation.y,
                      marker1_pose.pose.orientation.z,
                      marker1_pose.pose.orientation.w);
 tf::Matrix3x3(quat1).getRPY(mocap1_roll, mocap1_pitch,  mocap1_yaw);

    Q_B1.setZero();
     Q_B1<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
         2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
         2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;



}

double mocap2_roll , mocap2_yaw ,mocap2_pitch;
double x2, y2,z2;
void mocap2_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){

    marker2_pose =*msg;

    tf::Quaternion quat2(marker2_pose.pose.orientation.x,
                         marker2_pose.pose.orientation.y,
                         marker2_pose.pose.orientation.z,
                         marker2_pose.pose.orientation.w);
    tf::Matrix3x3(quat2).getRPY(mocap2_roll, mocap2_pitch,  mocap2_yaw);
    double x = marker2_pose.pose.orientation.x;
    double y = marker2_pose.pose.orientation.y;
    double z = marker2_pose.pose.orientation.z;
    double w = marker2_pose.pose.orientation.w;

    Q_B2.setZero();
     Q_B2<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
         2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
         2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "so3");
  ros::NodeHandle nh;

  //ros::Subscriber force_sub = nh.subscribe("/output", 10, force_cb);        //okay
  //ros::Subscriber tag_sub =  nh.subscribe("/fiducial_transforms", 10, tag_cb);

  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("point",10);

  ros::Subscriber imu1_sub = nh.subscribe("/imu1/mavros/imu/data", 10, imu1_cb);
  ros::Subscriber imu2_sub = nh.subscribe("/imu2/mavros/imu/data", 10, imu2_cb);

  ros::Subscriber marker2_sub = nh.subscribe("/vrpn_client_node/RigidBody4/pose", 10, mocap1_cb);  //okay
  ros::Subscriber marker1_sub = nh.subscribe("/vrpn_client_node/RigidBody1/pose", 10, mocap2_cb);
  ros::Subscriber force_sub = nh.subscribe("/follower_ukf/output", 10, force_cb);
  ros::Publisher leader_force_pub = nh.advertise<geometry_msgs::Point>("leader_force",2);
  ros::Rate loop_rate(50);
  forceest forceest1(statesize,measurementsize);
  Eigen::MatrixXd mnoise;
  mnoise.setZero(measurementsize,measurementsize);
  mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);

  mnoise(mrpx,mrpx) =  2e-3;
  mnoise(mrpy,mrpy) =  2e-3;
  mnoise(mrpz,mrpz) =  2e-3;
  mnoise(malphap,malphap) =  2e-2;
  mnoise(momegap,momegap) =  2e-2;
  mnoise(malphar,malphar) =  2e-2;
  mnoise(momegar,momegar) =  2e-2;
  mnoise(malphay,malphay) =  2e-2;
  mnoise(momegay,momegay) =  2e-2;
  mnoise(mac_x,mac_x) =  3e-3;
  mnoise(mac_y,mac_y) =  3e-3;
  mnoise(mac_z,mac_z) =  3e-3;
  mnoise(mFF_x,mFF_x) =  3e-3;
  mnoise(mFF_y,mFF_y) =  3e-3;
  mnoise(mFF_z,mFF_z) =  3e-3;

  forceest1.set_measurement_noise(mnoise);

  Eigen::MatrixXd pnoise;
  pnoise.setZero(statesize,statesize);
  pnoise(rp_x,rp_x) = 1e-2;
  pnoise(rp_y,rp_y) = 1e-2;
  pnoise(rp_z,rp_z) = 1e-2;

  pnoise(alpha_r , alpha_r) = 4e-2;
  pnoise(omega_r , omega_r) = 4e-2;
  pnoise(alpha_p , alpha_p) = 4e-2;
  pnoise(omega_p , omega_p) = 4e-2;
  pnoise(alpha_y , alpha_y) = 4e-2;
  pnoise(omega_y , omega_y) = 4e-2;

  pnoise(ac_x , ac_x) = 2e-2;
  pnoise(ac_y , ac_y) = 2e-2;
  pnoise(ac_z , ac_z) = 2e-2;

  pnoise(ap_x , ap_x) = 1e-2;
  pnoise(ap_y , ap_y) = 1e-2;
  pnoise(ap_z , ap_z) = 1e-2;

  pnoise(FF_x , FF_x) = 3e-1;
  pnoise(FF_y , FF_y) = 3e-1;
  pnoise(FF_z , FF_z) = 3e-1;

  pnoise(FL_x , FL_x) = 3e-1;
  pnoise(FL_y , FL_y) = 3e-1;
  pnoise(FL_z , FL_z) = 3e-1;

  forceest1.set_process_noise(pnoise);


  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,statesize);

  measurement_matrix(mrpx , rp_x) =1;
  measurement_matrix(mrpy , rp_y) =1;
  measurement_matrix(mrpz , rp_z) =1;

  measurement_matrix(momegar , omega_r) =1;
  measurement_matrix(momegap , omega_p) =1;
  measurement_matrix(momegay , omega_y) =1;

  measurement_matrix(malphar , alpha_r) =1;
  measurement_matrix(malphap , alpha_p) =1;
  measurement_matrix(malphay , alpha_y) =1;

  measurement_matrix(mac_x , ac_x) =1;
  measurement_matrix(mac_y , ac_y) =1;
  measurement_matrix(mac_z , ac_z) =1;


  measurement_matrix(mFF_x  , FF_x) =1;
  measurement_matrix(mFF_y  , FF_y) =1;
  measurement_matrix(mFF_z  , FF_z) =1;

  forceest1.set_measurement_matrix(measurement_matrix);
   std::cout << "====================" <<std::endl;
   std::cout<< "measurement matrix"<< std::endl  << measurement_matrix<<std::endl;
   std::cout << "====================" <<std::endl;
   std::cout<< "p matrix"<< std::endl  << pnoise<<std::endl;
   std::cout << "====================" <<std::endl;
   std::cout<< "Q matrix"<< std::endl  << mnoise<<std::endl;


   while(ros::ok()){

     forceest1.predict();

     Eigen::VectorXd measure;

     measure.setZero(measurementsize);


     dx = marker2_pose.pose.position.x - marker1_pose.pose.position.x;
     dy = marker2_pose.pose.position.y - marker1_pose.pose.position.y;
     dz = marker2_pose.pose.position.z - marker1_pose.pose.position.z;

     r_p<< dx,dy,dz;

     measure << dx , dy , dz ,
                w_I1[0] , w_I1[1] , w_I1[2],
                alpha[0],alpha[1] , alpha[2],
                a_I1[0] ,a_I1[1]   ,a_I1[2] ,
                Follower_Force.force.x , Follower_Force.force.y ,Follower_Force.force.z;
     forceest1.correct(measure);
      leader_force.x = forceest1.x[FL_x];
      leader_force.y = forceest1.x[FL_y];
      leader_force.z = forceest1.x[FL_z];

     Eigen::Vector3d a_p;

     a_p =  a_I1 - w_I1.cross(w_I1.cross(r_p)) - alpha.cross(r_p);

//     std::cout <<  "=====1=======" <<std::endl;
//     std::cout <<   forceest1.x  <<std::endl;
//     std::cout <<  "=====2=======" <<std::endl;

//     std::cout <<   a_I2 <<std::endl;

//     std::cout <<   forceest1.x[ap_x]  <<std::endl;
//     std::cout <<   forceest1.x[ap_y]  <<std::endl;
//     std::cout <<   forceest1.x[rp_z]  <<std::endl;
     point.x = forceest1.x[ap_z] ;
     point.y = a_p[2];
     point.z = a_I2[2];


     point_pub.publish(point);
     leader_force_pub.publish(leader_force);
     loop_rate.sleep();
     ros::spinOnce();
  }


  ROS_INFO("Hello world!");
}
