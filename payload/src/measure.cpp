#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "UKF/output.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <math.h>
#include "geometry_msgs/Transform.h"

std_msgs::Float64MultiArray measure;
//measurement


double pc_x=0.0 , pc_y=0.0, pc_z=0.0;
double theta_c=0.0 , theta_p=0.0;
double F_Fx =0.0 ,F_Fy =0.0 ,F_Fz=0.0;

//Eigen::VectorXd real_pose , relative_pose;




geometry_msgs::PoseStamped mocap_pose;
 double roll, pitch, yaw;


double fx=0.0 , fy=0.0 ,fz=0.0;
double ps_x =0.0,ps_y=0.0,ps_z=0.0;
geometry_msgs::PoseStamped marker_pose;
void marker_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

  double marker_roll ,marker_yaw , marker_pitch;


  marker_pose =*msg;


  tf::Quaternion quat1(marker_pose.pose.orientation.x,marker_pose.pose.orientation.y, marker_pose.pose.orientation.z, marker_pose.pose.orientation.w);
  tf::Matrix3x3(quat1).getRPY(marker_roll, marker_pitch,  marker_yaw);



//  std::cout <<"*****"<<std::endl;

//  std::cout << "roll" <<marker_roll<<std::endl;

//  std::cout << "yaw" <<marker_yaw<<std::endl;

//  std::cout << "pitch" <<marker_pitch<<std::endl;

//  std::cout <<"*****"<<std::endl;
}

double mocap_roll, mocap_yaw , mocap_pitch;
double w ,x ,y ,z;
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
   mocap_pose= *msg;
fx = mocap_pose.pose.position.x;
fy = mocap_pose.pose.position.y;
fz = mocap_pose.pose.position.z;

tf::Quaternion quat1(mocap_pose.pose.orientation.x,mocap_pose.pose.orientation.y, mocap_pose.pose.orientation.z, mocap_pose.pose.orientation.w);
tf::Matrix3x3(quat1).getRPY(mocap_roll, mocap_pitch,  mocap_yaw);

x = mocap_pose.pose.orientation.x;
y = mocap_pose.pose.orientation.y;
z = mocap_pose.pose.orientation.z;
w = mocap_pose.pose.orientation.w;


//std::cout <<"*****"<<std::endl;

//std::cout << "roll" <<mocap_roll<<std::endl;

//std::cout << "yaw" <<mocap_yaw<<std::endl;

//std::cout << "pitch" <<mocap_pitch<<std::endl;

//std::cout <<"*****"<<std::endl;

}
sensor_msgs::Imu imu_data;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg ){
  imu_data = *msg;
  tf::Quaternion quat1(imu_data.orientation.x,imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw);
}

ukf::output force;
void force_cb(const ukf::output::ConstPtr& msg){
  force =*msg;

      //ensure thate theta_c will not go to zero
  if(force.force.x > 0.01){
    // theta_c = atan(force.force.z / force.force.x);

  }

}

double tag_x,tag_y,tag_z;

double tag_roll,tag_yaw,tag_pitch;
geometry_msgs::Pose tag_pose;

fiducial_msgs::FiducialTransformArray tag_array;
geometry_msgs::Transform transform;
void tag_cb(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
  tag_array = *msg;
  Eigen::MatrixXd Rx , Ry,Rz;


  if(tag_array.transforms.size()>0){


    tag_x = tag_array.transforms[0].transform.translation.x ;
    tag_y = -1*tag_array.transforms[0].transform.translation.y ;
    tag_z = -1*tag_array.transforms[0].transform.translation.z ;

    Eigen::VectorXd r ,s;
    r.setZero(3);
    s.setZero(3);
    r<< tag_x ,tag_y ,tag_z;
    Rz.setZero(3,3);
     Rx.setZero(3,3);
      Ry.setZero(3,3);
//    std::cout<<"----------------------------- " <<std::endl;
//    std::cout<<"x "<<tag_x <<std::endl;
//    std::cout<<"y "<<tag_y <<std::endl;
//    std::cout<<"z "<<tag_z <<std::endl;
//    std::cout<<"+++++++++++++++++++++++++++++++++" <<std::endl;
    tf::Quaternion quatt(tag_array.transforms[0].transform.rotation.x,tag_array.transforms[0].transform.rotation.y, tag_array.transforms[0].transform.rotation.z, tag_array.transforms[0].transform.rotation.w);
    tf::Matrix3x3(quatt).getRPY(tag_roll, tag_pitch, tag_yaw);
    std::cout <<"*****"<<std::endl;

    std::cout << "tag_roll" <<tag_roll*180/3.14159<<std::endl;

    std::cout << "  tag_yaw" <<tag_yaw*180/3.14159<<std::endl;

    std::cout << " tag_pitch" <<tag_pitch*180/3.14159<<std::endl;

    std::cout <<"*****"<<std::endl;
    double c_r = cos(tag_roll) , s_r = sin(tag_roll);
    double c_p = cos(tag_pitch) , s_p = sin(tag_pitch);
    double c_y = cos(tag_yaw) , s_y = sin(tag_yaw);

    Rz <<
          c_y,-1*s_y,0,
          s_y,c_y,0,
          0,0,1;
    Rx << 1,0,0,
          0,c_r,-s_r,
          0,s_r,c_r;
    Ry<<c_p ,0, s_p,
        0 ,1,0,
        -1*s_p,0,c_p;



    s= Rz*Rx*Ry* r;
//    std::cout <<"s"<<std::endl<<s<<std::endl;
//     std::cout <<"r"<<std::endl<<r<<std::endl;

  }else
  {
    tag_x = tag_x;
    tag_y = tag_y;
    tag_z = tag_z;
    tag_roll  = tag_roll;
    tag_yaw = tag_yaw;
    tag_pitch = tag_pitch;


  }

}







void transform_toinertial(double roll, double yaw, double pitch){

    Eigen::MatrixXd Rx , Ry,Rz,Q;
    Eigen::VectorXd r , v,s;
  //  std::cout <<"*****"<<std::endl;

//    std::cout << "roll = " <<roll<<std::endl;

//    std::cout << "yaw = " <<yaw<<std::endl;

//    std::cout << "pitch = " <<pitch<<std::endl;
//
  //  std::cout <<"*****"<<std::endl;

    double c_r = cos(roll) , s_r = sin(roll);
    double c_p = cos(pitch) , s_p = sin(pitch);
    double c_y = cos(yaw) , s_y = sin(yaw);


    r.setZero(3);
    v.setZero(3);
    s.setZero(3);
    Rx.setZero(3,3);
    Ry.setZero(3,3);
    Rz.setZero(3,3);
    Q.setZero(3,3);
    r<< tag_x ,tag_y ,tag_z;
    Rz <<
          c_y,-1*s_y,0,
          s_y,c_y,0,
          0,0,1;
    Rx << 1,0,0,
          0,c_r,-s_r,
          0,s_r,c_r;
    Ry<<c_p ,0, s_p,
        0 ,1,0,
        -1*s_p,0,c_p;
    //wxyz
    //abcd
    Q<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,2*x*z+2*w*y,
        2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
        2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;




    s<<fx,fy,fz;
//     pc_x = fx + ps_x+ c_p*c_y * tag_x + (c_y*s_p*s_r-s_y*c_r) * tag_y + (c_y*s_p*c_r+s_y*s_r) * tag_z;
//     pc_y = fy + ps_y+ c_p*s_y * tag_x + (s_y*s_p+c_r*c_y) * tag_y     + (s_y*s_p*c_r-c_y*s_r) * tag_z;
//     pc_z = fz + ps_z+ -1*s_p  * tag_x + (s_r*c_p)     *tag_y          + (s_r*c_p)*tag_z;
     // std::cout<<s<<std::endl;

      v = s +Q*r;

     // std::cout <<"v" <<std::endl <<v<<std::endl;

      //Ry Rz Rx +

      pc_x = v(0);
      pc_y = v(1);
      pc_z= v(2);

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "measure");
  ros::NodeHandle nh;

  ros::Subscriber force_sub = nh.subscribe("/output", 10, force_cb);        //okay
  ros::Subscriber imu_sub = nh.subscribe("/drone1/mavros/imu/data", 10, imu_cb);
  ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/RigidBody3/pose", 10, mocap_cb);  //okay
  ros::Subscriber marker_sub = nh.subscribe("/vrpn_client_node/RigidBody2/pose", 10, marker_cb);
  ros::Subscriber tag_sub =  nh.subscribe("/fiducial_transforms", 10, tag_cb);

  ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/mocap/pose",10);
  ros::Publisher measure_pub = nh.advertise<std_msgs::Float64MultiArray>("measurement",10);


  ros::Publisher tag_pub = nh.advertise<geometry_msgs::Pose>("tag",10);
    //real_pose.setZero(3);
  ros::Rate loop_rate(30);
  while(ros::ok()){


   // real_pose = rotate(roll,pitch,yaw) *  relative_pose;

    theta_p = tag_pitch + pitch ;
    //RPY
    //PRY
    //
    transform_toinertial(mocap_roll,mocap_yaw ,mocap_pitch);  //get pcx pcy pcz

    theta_c = atan(force.force.z / force.force.x);

    F_Fx =  force.force.x ;
    F_Fy =  force.force.y ;
    F_Fz =  force.force.z ;



//    printf("-----------------------------\n");
//    printf("roll %f \n",roll);
//    printf("yaw %f \n",yaw);
//    printf("pitch %f \n",pitch);
//    printf("++++++++++++++++++++++++++++++\n");

    measure.data.clear();

    tag_pose.position.x =  pc_x;
    tag_pose.position.y =  pc_y;
    tag_pose.position.z =  pc_z;

    tag_pub.publish(tag_pose);
    measure_pub.publish(measure);
    loop_rate.sleep();
    mocap_pub.publish(mocap_pose);
    ros::spinOnce();

  }


  return 0;
}

