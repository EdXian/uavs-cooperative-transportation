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
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

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
//std::vector<trajectory_profile> data;
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


geometry_msgs::PoseStamped desired_pose;
geometry_msgs::Point record_pose;
geometry_msgs::Point desired_force;
geometry_msgs::Point desired_velocity;
geometry_msgs::Point feedforward;
double tt;
//trajectory_msgs::MultiDOFJointTrajectory traj;
trajectory_msgs::MultiDOFJointTrajectoryPoint point;

nav_msgs::Path  path;

bool planner_ready = false;

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
void force2_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    wrench2 = *msg;

    Eigen::Vector3d wrench,data;
    wrench << msg->wrench.force.x ,  msg->wrench.force.y , msg->wrench.force.z;
       data             =payload_Rotation*wrench;

    T_F <<data(0),data(1),0  ;// data(0),data(1),data(2);

    wrench2.wrench.force.x= data(0);
    wrench2.wrench.force.y= data(1);
    wrench2.wrench.force.z= data(2);

}
void force1_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
     //wrench1 = *msg;
     Eigen::Vector3d wrench,data;
     wrench << msg->wrench.force.x ,  msg->wrench.force.y , msg->wrench.force.z;
        data=payload_Rotation.transpose()*wrench;
     wrench1.wrench.force.x= data(0);
     wrench1.wrench.force.y= data(1);
     wrench1.wrench.force.z= data(2);
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

}


std::vector<trajectory_profile> planning_path(nav_msgs::Path path_data){

    std::vector<trajectory_profile> data_;
    double dt=0.5;
    double T=0;
    Eigen::VectorXd C_x,C_y , theta_x,theta_y;
    Eigen::MatrixXd A_x, A_y;
    std::vector<nav_msgs::Path> seg_vec;
    nav_msgs::Path tmp_path;
    std::vector <Eigen::VectorXd> v_x,v_y;
    int seg_size = 10;
    seg_vec.clear();
    int count = 0;
    int index = 0;
//    inverse the waypoints in path.

    for(unsigned int i=0;i< path.poses.size();i++){
        tmp_path.poses.push_back(path.poses[path.poses.size()-1-i]);
    }

    path = tmp_path;
    //split the path into 10 segments.
    A_x.setZero(path.poses.size(),6); //sixth order polynomial
    A_y.setZero(path.poses.size(),6);
    C_x.setZero(path.poses.size());
    C_y.setZero(path.poses.size());
    T=0;
    for(unsigned int i=0;i<path.poses.size();i++){
        double t_k=1;
        for(unsigned int j=0;j<6;j++){
            A_x(i,j) = t_k;
            t_k = t_k* T;
        }
        T+=dt;
    }
    A_y = A_x;
    for(unsigned int j=0;j<path.poses.size();j++){
        C_x(j) = path.poses[path.poses.size()-1-j].pose.position.x;
        C_y(j) = path.poses[path.poses.size()-1-j].pose.position.y;
    }
    theta_x = (A_x.transpose()*A_x).inverse()*A_x.transpose()*C_x;
    theta_y = (A_y.transpose()*A_y).inverse()*A_y.transpose()*C_y;

    double endtime = path.poses.size() *dt;
    data_.resize(2000);
    //compute desired position
    for(unsigned int i=0; i<2000; i++){
        double t = (2000-i) * (endtime/(2000-0));
        double x =  theta_x(0)+
                    theta_x(1)*t+
                    theta_x(2)*t*t+
                    theta_x(3)*t*t*t+
                    theta_x(4)*t*t*t*t+
                    theta_x(5)*t*t*t*t*t;
        double y =  theta_y(0)+
                    theta_y(1)*t+
                    theta_y(2)*t*t+
                    theta_y(3)*t*t*t+
                    theta_y(4)*t*t*t*t+
                    theta_y(5)*t*t*t*t*t;
        data_[i].pos(0) = x;
        data_[i].pos(1) = y;

    }

    //compute desired velocity
    for(unsigned int i=0;i<1000;i++){
        double t = (1000-i)* (endtime/(1000-0));
        double x =  1*theta_x(1)+
                    2*theta_x(2)*t+
                    3*theta_x(3)*t*t+
                    4*theta_x(4)*t*t*t+
                    5*theta_x(5)*t*t*t*t ;

        double y =  1*theta_x(1)+
                    2*theta_x(2)*t+
                    3*theta_x(3)*t*t+
                    4*theta_x(4)*t*t*t+
                    5*theta_x(5)*t*t*t*t ;
//        data_[i].vel(0) = x;
//        data_[i].vel(1) = y;
    }
    return data_;
}

qptrajectory plan;

path_def path_vec;


nav_msgs::Path inv_path;
void path_cb(const nav_msgs::Path::ConstPtr& msg){
}

trajectory_msgs::MultiDOFJointTrajectoryPoint traj;

void traj_cb(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg ){
    traj = *msg;
    if(traj.transforms.size()>0){
        planner_ready = true;
        ROS_WARN("poses size :  %d",traj.transforms.size());
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geo");
  ros::NodeHandle nh;
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
  //ros::Subscriber path_sub = nh.subscribe("/sPath" , 1, path_cb);
  ros::Subscriber point2_sub = nh.subscribe("pointpc2",2,point2_cb);
  ros::Subscriber point_sub = nh.subscribe("pointvc2",2,point_cb);
  ros::Subscriber traj_sub = nh.subscribe("/obstacle_avoidance_path",1,traj_cb);


  geometry_msgs::PoseStamped force;

  ros::Rate loop_rate(50.0);

  nh.setParam("/start",false);

    //get the waypoints




   Eigen::Vector4d output;


   desired_pose.pose.position.x = 0.9;
   desired_pose.pose.position.y = 0.0;
   desired_pose.pose.position.z = 1.3;

std::vector<trajectory_profile> data;


    while(ros::ok()){

    if(planner_ready){
        data.clear();
        if(traj.transforms.size()>0)
        {
            data.resize(traj.transforms.size());
            for(unsigned int i=0; i<traj.transforms.size();i++){

                data[i].pos << traj.transforms[i].translation.x, traj.transforms[i].translation.y, 0;
                data[i].vel << traj.velocities[i].linear.x, traj.velocities[i].linear.y, 0 ;
//                data[i].acc << traj.accelerations[i].linear.x, traj.accelerations[i].linear.y, 0;


                data[i].acc <<   0,0,0;
                data[i].jerk << 0, 0, 0;
                std::cout << i <<std::endl;
                std::cout << data[i].vel.transpose()<<std::endl;
            }
        }

        planner_ready = false;
    }



    nh.getParam("/start",flag);

    if(flag == false || (tick>data.size())){

       //do position control
       Eigen::Vector3d p,v,a;
       p<<desired_pose.pose.position.x,desired_pose.pose.position.y,desired_pose.pose.position.z;
       v<<0.0,0.0,0.0;
       a<<0.0,0.0,0.0;
        nh.setParam("/start",false);


       if(tick>data.size()){

         desired_pose.pose.position.x = data[data.size()-1].pos(0);
         desired_pose.pose.position.y = data[data.size()-1].pos(1);
         desired_pose.pose.position.z = 1.3;
         force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0))+1*(0-vel(0));
         force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1))+1*(0-vel(1));
         force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2))+1*(0-vel(2))+0.5*9.8*0.5;
         tick = 0;

       }else{
         force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0))+1*(0-vel(0));
         force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1))+1*(0-vel(1));
         force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2))+1*(0-vel(2))+0.5*9.8*0.5;
       }



     }else{
        // running the proposed controller
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
       std::cout << data[tick].pos.transpose()<<std::endl;


        if(theta_r <0 ) theta_r+=2*PI;

//              std::cout << "theta_r  " << theta_r <<std::endl;
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

       nonlinearterm = R_c2_B*(omega_m.cross(v_p))-omega_m.cross(omega_m.cross(r_c2_p));
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

        tmp <<  2.0 * (nonholoutput(0) - vc2_est(0))+ err_state_B(0) + 1.0*(nonlinearterm(0) + vd_dot) ,
             4.0 * (nonholoutput(1)-vc2_est(2)) + sin(theta_e)/1.0 +omegad_dot  ,
                                                  0;

        feedforward.x =nonlinearterm(0);
        feedforward.y = vd_dot;
        feedforward.z = omegad_dot;

        cmd_ = R_c2_B.transpose() *  tmp;

        tick++;
        //
        vp_dot_des(0) =  cmd_(0);// + nonlinearterm(0);// + vd_dot ;
        vp_dot_des(1) =  cmd_(1);// + omegad_dot;
        vp_dot_des(2) = 3*(1.3 - p_p(2))+0.5*(0-v_p(2));

        T_F(0) = 0;
        T_F(1) = 0;

        T_L = -T_F + mp * vp_dot_des ;



//         T_L = + mp * vp_dot_des ;
        desired_force.x = T_L(0);
        desired_force.y = T_L(1);
        desired_force.z = T_L(2);
        desired_velocity.x = v_c2_B(0);
        desired_velocity.y = v_c2_B(2);

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
