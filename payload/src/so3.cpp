#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "lpf.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>

double mocap1_roll , mocap1_yaw ,mocap1_pitch;
gazebo_msgs::ModelStates model;
gazebo_msgs::LinkStates links;
Eigen::Vector3d pc,pv,pa,pb;
sensor_msgs::Imu imu_data;
Eigen::Matrix3d payload_rotation;

void imu1_cb(const sensor_msgs::Imu::ConstPtr& msg){


Eigen::Vector3d  data;
imu_data = *msg;
data<<imu_data.linear_acceleration.x , imu_data.linear_acceleration.y ,imu_data.linear_acceleration.z ;

double w,x,y,z;
x=imu_data.orientation.x;
y=imu_data.orientation.y;
z=imu_data.orientation.z;
w=imu_data.orientation.w;

payload_rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
            2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
            2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


pa = payload_rotation * data - Eigen::Vector3d(0,0,9.8);

}
void imu2_cb(const sensor_msgs::Imu::ConstPtr& msg){

}

void link_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    links = *msg;

    if(links.name.size()>0){
        for(unsigned int i=0;i<links.name.size();i++){
            if(links.name[i] == "payload::payload_rec_g1_box"){

//                pc<<links.pose[i].position.x,
//                    links.pose[i].position.y,
//                    links.pose[i].position.z;
                pv<<links.twist[i].linear.x,
                    links.twist[i].linear.y,
                    links.twist[i].linear.z;
            }
            if(links.name[i] == "payload::payload_link2_box"){

                pb<<links.pose[i].position.x,
                    links.pose[i].position.y,
                    links.pose[i].position.z;

            }


        }
    }

}

void force_cb(const geometry_msgs::Point::ConstPtr& msg){
Eigen::Vector3d f;
double l =0.5;   //the length of the cable
f<<msg->x , msg->y , msg->z;
 pc = pb+(f/f.norm())*l;



}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "so3");
  ros::NodeHandle nh;

  ros::Subscriber imu1_sub = nh.subscribe("/payload/IMU1", 2, imu1_cb);
  ros::Subscriber link_sub = nh.subscribe("/gazebo/link_states",2,link_cb);
  ros::Subscriber force_sub = nh.subscribe("/leader_ukf/force_estimate",2,force_cb);


  ros::Publisher point2_pub = nh.advertise<geometry_msgs::Point>("point",2);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("pointso3",2);


  ros::Rate loop_rate(50);
  forceest forceest1(statesize,measurementsize);
  Eigen::MatrixXd mnoise;

  mnoise.setZero(measurementsize,measurementsize);
  mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);
  mnoise(mpc1_x,mpc1_x) =  2e-3;
  mnoise(mpc1_y,mpc1_y) =  2e-3;
  mnoise(mpc1_z,mpc1_z) =  2e-3;

  mnoise(mac1_x,mac1_x) =  2e-3;
  mnoise(mac1_y,mac1_y) =  2e-3;
  mnoise(mac1_z,mac1_z) =  2e-3;

  forceest1.set_measurement_noise(mnoise);

  Eigen::MatrixXd pnoise;
  pnoise.setZero(statesize,statesize);
    pnoise(pc1_x,pc1_x) = 1e-2;
    pnoise(pc1_y,pc1_y) = 1e-2;
    pnoise(pc1_z,pc1_z)  = 1e-2;

    pnoise(vc1_x,vc1_x) = 1e-2;
    pnoise(vc1_y,vc1_y) = 1e-2;
    pnoise(vc1_z,vc1_z)  = 1e-2;

    pnoise(ac1_x,ac1_x) = 1e-2;
    pnoise(ac1_y,ac1_y) = 1e-2;
    pnoise(ac1_z,ac1_z)  = 1e-2;

  forceest1.set_process_noise(pnoise);


  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,statesize);

  measurement_matrix(mpc1_x, pc1_x )  =1 ;
  measurement_matrix(mpc1_y, pc1_y ) = 1;
  measurement_matrix(mpc1_z, pc1_z ) =1 ;

  measurement_matrix(mac1_x, ac1_x )  =1 ;
  measurement_matrix(mac1_y, ac1_y ) = 1;
  measurement_matrix(mac1_z, ac1_z ) =1 ;


  forceest1.set_measurement_matrix(measurement_matrix);

   while(ros::ok()){

     forceest1.predict();

     Eigen::VectorXd measure;




     measure.setZero(measurementsize);

     measure << pc(0),pc(1),pc(2),pa(0),pa(1),pa(2);



     forceest1.correct(measure);


     geometry_msgs::Point point ,point2;
    point.x = forceest1.x[vc1_x] ;
    point.y = forceest1.x[vc1_y];

    point2.x = pv(0);
    point2.y = pv(1);

     point_pub.publish(point);
     point2_pub.publish(point2);
     loop_rate.sleep();
     ros::spinOnce();
  }


  ROS_INFO("Hello world!");
}
