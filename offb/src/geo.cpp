#include <ros/ros.h>
#include <geometric_controller.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
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
#include <qptrajectory.h>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#define normal
#define PI 3.14159
gazebo_msgs::ModelStates model_states;
geometry_msgs::PoseStamped payload_pose , leader_pose;
geometry_msgs::TwistStamped leader_vel;
Eigen::Vector3d pose,vel,ang   ;
Eigen::Vector4d ori ;
Eigen::Vector3d v_p,v_p_B,p_p,v_omega_p;
Eigen::Vector3d p_c2,v_p_c2;
Eigen::Vector3d T_F ,T_L, T_F_est;
Eigen::Vector3d e_dot,e;
double vir_x, vir_y, theta_r,vr,omega_r ,vx ,vy ,ax,ay,jx,jy;
double last_theta_r;
double last_omega;

double payload_roll , payload_yaw, payload_pitch;
Eigen::Matrix3d payload_Rotation;
Eigen::Vector3d vc2_est;
Eigen::Vector3d pc2_est;
Eigen::Matrix3d payload_link2_Rotation;

double x_e,y_e ,theta_e    ;
Eigen::Vector3d err_state  ;
Eigen::Vector3d err_state_B;
unsigned int tick=0;
bool flag = false;
mavros_msgs::State current_state;

gazebo_msgs::LinkStates link_states;
Eigen::Matrix3d R_c2_B;
Eigen::Vector3d v_c2_I;
Eigen::Vector3d v_c2_B;


sensor_msgs::Imu imu_data;
void imu1_cb(const sensor_msgs::Imu::ConstPtr& msg){


Eigen::Vector3d  data;
imu_data = *msg;
data<<imu_data.linear_acceleration.x , imu_data.linear_acceleration.y ,imu_data.linear_acceleration.z ;

double w,x,y,z;
x=imu_data.orientation.x;
y=imu_data.orientation.y;
z=imu_data.orientation.z;
w=imu_data.orientation.w;
tf::Quaternion Q(
x,
y,
z,
w   );
tf::Matrix3x3(Q).getRPY(payload_roll,payload_pitch,payload_yaw);

payload_Rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                    2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                    2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


R_c2_B << cos(payload_yaw), sin(payload_yaw) ,0,
         -sin(payload_yaw), cos(payload_yaw) ,0,
             0    ,      0   ,1;

Eigen::Vector3d tmp;
tmp <<  imu_data.angular_velocity.x,
        imu_data.angular_velocity.y,
        imu_data.angular_velocity.z;

v_c2_I = (payload_Rotation)* tmp;
v_c2_B = R_c2_B * v_c2_I;
v_omega_p = v_c2_B;
}



Eigen::Vector3d nonholonomic_output(double x_r, double y_r,double theta_r,double v_r, double omega_r){


    Eigen::Vector3d output;
    double yaw = payload_yaw;//+(PI/2);
    err_state << x_r - pc2_est(0) , y_r - pc2_est(1) , theta_r - (yaw);
    Eigen::Matrix3d R_I_B;

    theta_e = theta_r- (yaw);
    err_state(2) = theta_e;

    R_I_B << cos(yaw) , sin(yaw) ,0,
             -sin(yaw) , cos(yaw),0,
            0         ,      0  , 1;

    err_state_B  = R_I_B * err_state;

    err_state_B(2) = 0;

    x_e = err_state_B(0);///err_state_B.norm();       ///err_state_B.norm();
    y_e = err_state_B(1);///err_state_B.norm();       ///err_state_B.norm();
    double v = v_r * cos(theta_e) + 1.0*x_e;  // 0.3

    double w = omega_r + v_r*(5.0*y_e+3.0*sin(theta_e));  //

    output <<v,w,0;
    return output ;
}


geometry_msgs::WrenchStamped wrench1;
geometry_msgs::Point est_force;
//geometry_msgs::Point est_force;

geometry_msgs::WrenchStamped wrench2;

Eigen::Matrix3d uav_rotation;
 Eigen::Vector3d pc;
void force_cb(const geometry_msgs::Point::ConstPtr &msg){

    Eigen::Vector3d f;
    double l =0.18;   //the length of the cable
    double L = 0.5; // the length of the payload
    f<<msg->x , msg->y , msg->z;
        Eigen::Vector3d pb;
     pb  =   pose    -  uav_rotation * Eigen::Vector3d(0,0,0.05);
     pc = pb+(f/f.norm())*l;




}

void est_force_cb(const geometry_msgs::Point::ConstPtr& msg){
    est_force = *msg;

}

void est_vel_cb(const geometry_msgs::Point::ConstPtr& msg){
    Eigen::Vector3d pc1= Eigen::Vector3d(msg->x,msg->y,msg->z);
    Eigen::Vector3d rcp = Eigen::Vector3d(-0.5,0.0,0.0);
    v_p = pc1 + v_c2_I.cross(rcp);

}

void point2_cb(const geometry_msgs::Point::ConstPtr& msg){

    pc2_est<<msg->x,msg->y,msg->z;
    p_p(2) = msg->z;
}
void point_cb(const geometry_msgs::Point::ConstPtr& msg){
    vc2_est <<  msg->x , msg->y, msg->z;

}

nav_msgs::Odometry odom;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;

   leader_pose.pose = odom.pose.pose;
   leader_vel.twist = odom.twist.twist;

    ori<< leader_pose.pose.orientation.w, leader_pose.pose.orientation.x,leader_pose.pose.orientation.y,leader_pose.pose.orientation.z;
    pose << leader_pose.pose.position.x , leader_pose.pose.position.y, leader_pose.pose.position.z;
    vel  << leader_vel.twist.linear.x, leader_vel.twist.linear.y, leader_vel.twist.linear.z;
   ang << leader_vel.twist.angular.x, leader_vel.twist.angular.y,leader_vel.twist.angular.z;

   double w,x,y,z;
   x=msg->pose.pose.orientation.x;
   y=msg->pose.pose.orientation.y;
   z=msg->pose.pose.orientation.z;
   w=msg->pose.pose.orientation.w;

   uav_rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
               2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
               2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;




}

geometry_msgs::PoseStamped desired_pose;
geometry_msgs::Point record_pose;
geometry_msgs::Point desired_force;
geometry_msgs::Point desired_velocity;
geometry_msgs::Point feedforward;
double tt;
trajectory_msgs::MultiDOFJointTrajectory traj;
trajectory_msgs::MultiDOFJointTrajectoryPoint point;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "geo");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");



   ros::Publisher feedforward_pub = nh.advertise<geometry_msgs::Point>("/feedforward", 2);
   ros::Publisher desired_pose_pub = nh.advertise<geometry_msgs::Point>("/drone1/desired_position", 2);
   ros::Publisher desired_force_pub = nh.advertise<geometry_msgs::Point>("/desired_force", 2);
   ros::Publisher desired_velocity_pub = nh.advertise<geometry_msgs::Point>("/desired_velocity",2);
   ros::Publisher   traj_pub= nh.advertise<geometry_msgs::PoseStamped>("/firefly1/command/pose", 2);
   ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
           ("/firefly1/odometry_sensor1/odometry", 3, odom_cb);
   ros::Subscriber est_vel_sub = nh.subscribe<geometry_msgs::Point>("est_vel",3,est_vel_cb);

  ros::Subscriber est_force_sub = nh.subscribe<geometry_msgs::Point>
          ("/follower_ukf/force_estimate", 3, est_force_cb);
  ros::Subscriber imu1_sub = nh.subscribe("/payload/IMU1", 2, imu1_cb);


  ros::Subscriber force_sub = nh.subscribe("/leader_ukf/force_estimate",2,force_cb);

  ros::Subscriber point2_sub = nh.subscribe("pointpc2",2,point2_cb);
  ros::Subscriber point_sub = nh.subscribe("pointvc2",2,point_cb);



   ros::Rate loop_rate(50.0);
   nh.setParam("/start",false);
   geometry_msgs::PoseStamped force;

//planning
   qptrajectory plan;
   path_def path;
   trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8 , p9 , p10,p11;
   std::vector<trajectory_profile> data;

   p2.pos<< 1,1,0;
        p2.vel<< 0,0,0;
        p2.acc<< 0,0,0;
        p2.yaw = 0;

        p3.pos<< 3,5,0;
        p3.vel<< 0,0,0;
        p3.acc<< 0,0,0;
        p3.yaw = 0;

        p5.pos << 12,0,0;
        p5.vel << 0,0,0;
        p5.acc << 0,0,0;
        p5.yaw = 0;

        p6.pos << 3,-5,0;
        p6.vel << 0,0,0;
        p6.acc << 0,0,0;
        p6.yaw = 0;

        p8.pos<< -3,5,0;
        p8.vel<< 0,0,0;
        p8.acc<< 0,0,0;
        p8.yaw = 0;

        p9.pos << -12,0,0;
        p9.vel << 0,0,0;
        p9.acc << 0,0,0;
        p9.yaw = 0;

        p10.pos << -3,-5,0;
        p10.vel << 0,0,0;
        p10.acc << 0,0,0;
        p10.yaw = 0;

        p11.pos << 0,0,0;
        p11.vel << 0,0,0;
        p11.acc << 0,0,0;
        p11.yaw = 0;
   // path.push_back(segments(p1,p2,16.0));
    path.push_back(segments(p2,p3,16.0));
    path.push_back(segments(p3,p5,16.0));

    path.push_back(segments(p5,p6,16.0));
    path.push_back(segments(p6,p8,16.0));
   // path.push_back(segments(p7,p8,16.0));
    path.push_back(segments(p8,p9,16.0));
    path.push_back(segments(p9,p10,16.0));
    path.push_back(segments(p10,p11,16.0));
   data = plan.get_profile(path , path.size(),0.02);


   Eigen::Vector4d output;


   desired_pose.pose.position.x = 0.9;
   desired_pose.pose.position.y = 0.0;
   desired_pose.pose.position.z = 1.3;

       while(ros::ok()){


       if(nh.getParam("/start",flag)){

       };


     if(flag == false || (tick>data.size())){
               //do position control
               Eigen::Vector3d p,v,a;
               p<<desired_pose.pose.position.x,desired_pose.pose.position.y,desired_pose.pose.position.z;
               v<<0.0,0.0,0.0;
               a<<0.0,0.0,0.0;
                nh.setParam("/start",false);

                tick = 0;

                force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0))+1*(0-vel(0));
                force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1))+1*(0-vel(1));
                force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2))+1*(0-vel(2))+0.5*9.8*0.5;


           }else{

               vir_x = data[tick].pos(0);
               vir_y = data[tick].pos(1);
               record_pose.x = vir_x;
               record_pose.y = vir_y;

               vx = data[tick].vel(0);
               vy = data[tick].vel(1);
               ax = data[tick].acc(0);
               ay = data[tick].acc(1);
               jx = data[tick].jerk(0);
               jy = data[tick].jerk(1);

               theta_r = atan2( data[tick].vel(1),data[tick].vel(0) );

               if(theta_r <0 ){
                   theta_r+=2*PI;
               }


//               std::cout << "theta_r  " << theta_r <<std::endl;
       Eigen::Vector3d alpha ;
       alpha << 0,0, (v_c2_I(2) - last_omega )/0.02;
       last_omega = v_c2_I(2);

       double omega_r = (ay*vx - ax*vy)/(vx*vx+vy*vy); //(theta_r - last_theta_r) /(0.02) ;
       last_theta_r = theta_r;
       double vr = sqrt(vx*vx+ vy*vy);
       double vr_dot = sqrt(ax*ax+ay*ay);
       double theta_e = theta_r - payload_yaw;
       double theta_e_dot =omega_r - v_omega_p(2);  //the error of the angular velocity
       double x_e_dot = v_omega_p(2) * err_state_B(1) +vr*cos(theta_e);
       double y_e_dot = -v_omega_p(2) *  err_state_B(0) + vr*sin(theta_e);
       double omegar_dot = (jy*vx - jx*vy)/(vr*vr) - (2*vr_dot*omega_r)/vr ;

       double omegad_dot= omegar_dot + vr_dot*1*err_state_B(1) +vr*1*y_e_dot + 1*theta_e_dot*cos(theta_e);
       double vd_dot = vr_dot*cos(theta_e) - vr*theta_e_dot*sin(theta_e) + 1* x_e_dot;

       Eigen::Vector3d omega_m;
       omega_m << 0 , 0 , v_c2_I(2);
       Eigen::Vector3d   r_c2_p;
       Eigen::Vector3d   nonlinearterm;
       r_c2_p  << 0.5 , 0,0;


       nonlinearterm = R_c2_B*(omega_m.cross(v_p))- alpha.cross(r_c2_p)-omega_m.cross(omega_m.cross(r_c2_p));
       Eigen::Vector3d nonholoutput = nonholonomic_output(vir_x,vir_y,theta_r,vr,omega_r);

       //  R_-1 * omega x v - omega_dot x rcp - omega x (omega x r)

       Eigen::Vector3d vp_dot_des;

       T_F(2) = 0.5*(0.5*-9.8);    //1/2 weight of the payload.

       if( nonholoutput(0) > 10 ){
           nonholoutput(0)= 10;
       }

        Eigen::Vector2d cmd ;

        Eigen::Vector3d tmp;
        Eigen::Vector3d cmd_;
        double mp=0.5;
       // std::cout << "ok"<<std::endl;

        double L = 1.0;
        double I_p = (0.0833)*0.5*1.0*1.0;

        tmp <<  3.0 * (nonholoutput(0) - vc2_est(0))+ err_state_B(0) + nonlinearterm(0) + vd_dot ,
             3.0 * (nonholoutput(1)-vc2_est(2)) + sin(theta_e)/1.0 +omegad_dot  ,   //ffy is close to zero.
                                                  0;

        feedforward.x =nonlinearterm(0);
        feedforward.y = vd_dot;
        feedforward.z = omegad_dot;

        cmd_ = R_c2_B.transpose() *  tmp;

        tick++;
        //
        vp_dot_des(0) =  cmd_(0);// + nonlinearterm(0);// + vd_dot ;
        vp_dot_des(1) =  cmd_(1);// + omegad_dot;
        vp_dot_des(2) = 1.0*(1.3 - pose(2))+0.6*(0-vel(2));

        Eigen::Matrix3d M;
        M<<     mp,0,0,
                0,0.666,0,
                0,0,1;
        T_L = -T_F + M * vp_dot_des ;

        desired_force.x = T_L(0);
        desired_force.y = T_L(1);
        desired_force.z = T_L(2);


        desired_velocity.x = vr;
        desired_velocity.y = omega_r;



        force.pose.position.x = T_L(0);
        force.pose.position.y = T_L(1);
        force.pose.position.z = T_L(2);

           }

           traj_pub.publish(force);
           desired_pose_pub.publish(record_pose);
           desired_force_pub.publish(desired_force);
           desired_velocity_pub.publish(desired_velocity);
           feedforward_pub.publish(feedforward);
           ros::spinOnce();
           loop_rate.sleep();

       }


}
