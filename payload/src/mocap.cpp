#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "UKF/output.h"
#include "geometry_msgs/Point.h"
#include "lpf.h"
#include "kalmanfilter.h"
#include "eigen3/unsupported/Eigen/FFT"

#include "fiducial_msgs/FiducialTransformArray.h"


fiducial_msgs::FiducialTransformArray dataarray;



sensor_msgs::Imu imu_data;
geometry_msgs::PoseStamped mocap_pose;

Eigen::Vector3d a_I , a_B;
UKF::output  ukf_data;


double fx ,fy ,fz;
double mocap_roll , mocap_yaw , mocap_pitch;
double theta_p;
double omega_p;
bool flag = true;
double yaw_bias ;
double imu_roll , imu_pitch , imu_yaw;
double x_bias , y_bias , z_bias;
double a_x_I,a_y_I,a_z_I;
double a_x_B,a_y_B,a_z_B;
bool imu_flag=true;
double w,x,y,z;
double last_omega_p ;
Eigen::Matrix3d R_B_I ; //body frame to inertial frame matrix
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  a_B.setZero(3);
  imu_data = *msg;
  a_B<<imu_data.linear_acceleration.x ,
       imu_data.linear_acceleration.y ,
       imu_data.linear_acceleration.z ;

//last_omega_p = omega_p;
  omega_p = imu_data.angular_velocity.y;
  tf::Quaternion quat1(imu_data.orientation.x,imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3(quat1).getRPY(imu_roll, imu_pitch,  imu_yaw);
  //last_theta_p = theta_p;
  theta_p = imu_pitch;

  //correct yaw angle bias in body frame
  if(flag){
    yaw_bias = imu_yaw;
    x_bias = imu_data.linear_acceleration.x;
    y_bias = imu_data.linear_acceleration.y;
    z_bias = imu_data.linear_acceleration.z;
    flag = false;
  }


  if( (imu_yaw - yaw_bias) < 0  ){

       imu_yaw = imu_yaw - yaw_bias +2*3.14159 ;
  }else{

    imu_yaw  = imu_yaw- yaw_bias;
  }
}
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

  Eigen::Matrix3d Rx ,Ry,Rz ,Q;
  mocap_pose= *msg;
fx = mocap_pose.pose.position.x;
fy = mocap_pose.pose.position.y;
fz = mocap_pose.pose.position.z;

x = mocap_pose.pose.orientation.x;
y = mocap_pose.pose.orientation.y;
z = mocap_pose.pose.orientation.z;
w = mocap_pose.pose.orientation.w;

tf::Quaternion quat1(mocap_pose.pose.orientation.x,mocap_pose.pose.orientation.y, mocap_pose.pose.orientation.z, mocap_pose.pose.orientation.w);
tf::Matrix3x3(quat1).getRPY(mocap_roll, mocap_pitch,  mocap_yaw);

if(mocap_yaw <0){
 mocap_yaw += 2*3.14159;
}


  Q<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,2*x*z+2*w*y,
      2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
      2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;
  a_I = Q*a_B;
  a_x_I = a_I[0];
  a_y_I = a_I[1];
  a_z_I = a_I[2] - 9.81;
}


sensor_msgs::Imu object;
Eigen::Vector3d  a_obj;
Eigen::Vector3d  omega_obj;
 Eigen::Vector3d att_obj;
double a_obj_x =0;
double a_obj_y = 0;
double a_obj_z = 0;
void imu2_cb(const sensor_msgs::Imu::ConstPtr& msg){
 Eigen::Matrix3d Q;

 object = *msg;
 Eigen::Vector3d data;

 tf::Quaternion quat1(object.orientation.x,object.orientation.y,object.orientation.z, object.orientation.w);

 tf::Matrix3x3(quat1).getRPY(att_obj[0], att_obj[1],  att_obj[2]);

 data.setZero(3);
   data  << object.linear_acceleration.x,
               object.linear_acceleration.y,
               object.linear_acceleration.z;

   Q<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,2*x*z+2*w*y,
       2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
       2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;



 a_obj = Q*data;
 last_omega_p = omega_obj[1] ;
  omega_obj[0] = object.angular_velocity.x;
  omega_obj[1] = object.angular_velocity.y;
  omega_obj[2] = object.angular_velocity.z;
 //std::cout << "Q"<<std::endl<<a_obj<<std::endl;

  a_obj_x=a_obj[0] ;
  a_obj_y=a_obj[1] ;
  a_obj_z=a_obj[2]-9.81 ;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe("/imu/mavros/imu/data" , 2, imu_cb);
  ros::Subscriber mocap_sub = nh.subscribe("/vrpn_client_node/RigidBody1/pose" , 2 , mocap_cb);
  geometry_msgs::Point point;
  ros::Subscriber imu2_sub = nh.subscribe("/imu2/mavros/imu/data" , 2, imu2_cb);

  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("theta_p",10);
  ros::Rate loop_rate(30);

  kalmanfilter kf(1,1);
  Eigen::MatrixXf A;
  Eigen::MatrixXf H;
  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;
  Eigen::MatrixXf P;
  Eigen::VectorXf v;

  v.setZero(1);
  A.setZero(1,1);
  H.setZero(1,1);
  Q.setZero(1,1);
  R.setZero(1,1);
  P.setZero(1,1);
  A<<1;
  H<<1;
  Q<<7e-3;
  R<<1e-2;
  P<<1e-4;
  kf.setFixed(A,H,Q,R);
  v<<0;
  kf.setInitial(v , P);


  lpf lpf1(5,0.033);
   lpf lpf2(5,0.033);
  double x ;
  double velocity=0.0;


  double last_x=0;
  double last_velocity=0;
  double accelerate;

  double time =ros::Time::now().toSec();
  double lastime =time;
  double delta;
  while(ros::ok()){

    time = ros::Time::now().toSec();
    delta = time - lastime;

    x= mocap_pose.pose.position.x;
    //
    //
  double acx = a_x_I;
  double acz = a_z_I;

  double alpha_p = (omega_obj[1] - last_omega_p)/delta;
  double r=0.6;
  // +alpha_p*r*cos(theta_p)
  // +alpha_p*r*sin(theta_p)


//    velocity = (x-last_x) /  delta;

//    accelerate = (velocity-last_velocity) /  delta;
//    kf.predict();
//    VectorXf Z;
//    Z.setZero(1);
//    Z<<velocity ;
//    kf.correct(Z);

//    velocity = kf.X[0];



    lpf1.Ts =  delta;
    lpf2.Ts =  delta;
   //alpha_p= lpf1.filter( alpha_p);
    //+alpha_p*r*cos(theta_p)
    //+alpha_p*r*sin(theta_p)
    double apx =acx+ alpha_p*r*sin(theta_p) - omega_obj[1]*omega_obj[1] *cos(att_obj[1]);
    double apz =acz+ alpha_p*r*cos(theta_p) + omega_obj[1]*omega_obj[1]*r *sin(att_obj[1]);

//theta_p -0.072
    point.x  = apx;
    point.y  = a_obj_x;
   //  point.z = mocap_pitch;
   // point.z  = omega_obj[1];
    //point.y  = a_obj[2];

   // point.z = kf.X[0];

    point_pub.publish(point);



    last_x = x;
    last_velocity = velocity;
    lastime = time;



    ros::spinOnce();
   loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
