#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "std_msgs/Float64.h"
#include <QMainWindow>
#include <QApplication>
#include<QTimer>
#include <QFileDialog>
#include <QString>
#include "ros/ros.h"
#include "include/rigidbody.h"
#include"stdio.h"
#include "math.h"
#include "geometry_msgs/TwistStamped.h"
#include "ui_mainwindow.h"
#include "sensor_msgs/BatteryState.h"
#include "mavros_msgs/State.h"
#include "string.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "vector"
#include "geometry_msgs/WrenchStamped.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "gazebo_msgs/ModelStates.h"
#include "nav_msgs/OccupancyGrid.h"
struct tracker_pos{
  QVector<double> x;
  QVector<double> y;
  QVector<double> z;
};


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();



private slots:
void on_checkBox_stateChanged(int arg1);
    void plot_loop();
    void state_loop();
void on_pushButton_clicked();
 void on_pushButton_2_clicked();
 void on_maxspinbox_valueChanged(const QString &arg1);
 void on_minspinbox_valueChanged(const QString &arg1);

private:
    Ui::MainWindow *ui;
//    QCPCurveData plot;

    QVector<double> pointx;
    QVector<double> pointy;
    QVector<double> pointz;
    QVector<double> point_time;
    QVector<double> point_time2;
    QCPCurve *path_curve;
    QVector<QCPCurveData> path_curve_data;

    QCPCurve *conn1_curve;
    QVector<QCPCurveData> conn1_curve_data;

    QCPCurve *conn2_curve;
    QVector<QCPCurveData> conn2_curve_data;


    QCPCurve *map_path_curve;
    QVector<QCPCurveData> map_path_curve_data;

    QCPCurve *desired_curve;
    QVector<QCPCurveData> desired_curve_data;

    QCPCurve *map_desired_curve;
    QVector<QCPCurveData> map_desired_curve_data;
//    QVector<QCPCurveData> curve_data;
//    std::vector<QCPCurve*> curve_list;
//    std::vector<QCPCurve*> path_curve_list;
//    QVector<QCPCurveData> path_data;

    void traj_cb(const  geometry_msgs::Point::ConstPtr& msg  );
    void desired_traj_cb(const geometry_msgs::Point::ConstPtr& msg);
    void point_cb(const geometry_msgs::Point::ConstPtr& msg);

    void acc_cb(const geometry_msgs::Point::ConstPtr& msg);
    void local_cb(const  geometry_msgs::Point::ConstPtr& msg  );
    void model_cb(const  gazebo_msgs::ModelStates::ConstPtr& msg  );
    void link_cb(const  gazebo_msgs::LinkStates::ConstPtr& msg  );

    void force1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void force2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void estimate_force_cb(const geometry_msgs::Point::ConstPtr& msg);
    void desired_force_cb(const geometry_msgs::Point::ConstPtr& msg);
    void trigger_cb(const geometry_msgs::Point::ConstPtr& msg);
    void desired_velocity_cb(const geometry_msgs::Point::ConstPtr& msg);
    void vel_est_cb(const  geometry_msgs::Point::ConstPtr& msg);
    void vel_est_b_cb(const geometry_msgs::Point::ConstPtr& msg);
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    //   void me_cb(const  ukf_estimate::output::ConstPtr& msg  );
    double t_max;
    double t_min;
    nav_msgs::OccupancyGrid map_grid_data;
    tracker_pos tracker;
    QTimer *timer;
    QTimer *state_timer;
    std::vector<QVector<double>> x,y;
    geometry_msgs::Point point ;
    geometry_msgs::WrenchStamped force ;
    geometry_msgs::Point desired_force ;

    geometry_msgs::Point estimate_force;
    geometry_msgs::Point force2;

    Eigen::Matrix3d  payload_link1_rotation;
    Eigen::Matrix3d payload_link2_rotation;

    geometry_msgs::Point payload_pose ;
    geometry_msgs::Point payload_vel ;

    geometry_msgs::Point leader_pose ;
    geometry_msgs::Point follower_pose ;

    geometry_msgs::Point c2_vel;
    geometry_msgs::Point c2_vel_b;
    Eigen::Matrix3d rotation_matrix;
    std::vector<QCPItemLine* > arrow;
    int tick;
    double ros_time = 0;
    double last_ros_time=0;

    double time = 0;
    double last_time = 0;
    double count=0;
    double count2=0;
    double count3=0;
    double count4 =0;
    double count5=0;
    ros::NodeHandle nh;
    ros::Subscriber point_sub;
    ros::Subscriber acc_sub;
    ros::Subscriber tracker1_sub;
    ros::Subscriber tracker2_sub;
    ros::Subscriber tracker3_sub;
    ros::Subscriber local_sub;
    ros::Subscriber traj_sub;
    ros::Subscriber trigger_sub;
    ros::Subscriber desired_traj_sub;
    ros::Subscriber model_state_sub;
    ros::Subscriber link_state_sub;
    ros::Subscriber estimate_force_sub;
    ros::Subscriber force2_sub;
    ros::Subscriber force1_sub;
    ros::Subscriber desired_force_sub;
    ros::Subscriber desired_vel_sub;
    ros::Subscriber map_sub;
    ros::Subscriber vel_est_sub;
    ros::Subscriber vel_estb_sub;

    ros::Publisher px;
    ros::Publisher py;

    ros::Publisher flx;
    ros::Publisher fly;

    ros::Publisher ffx;
    ros::Publisher ffy;

    ros::Publisher vc2_vx;
    ros::Publisher vc2_vy;

    ros::Publisher vc2_v;  //(est des groundtruth)
    ros::Publisher vc2_w;   //(est des groundtruth)
    ros::Publisher trig;

    ros::Publisher vcb;


    geometry_msgs::Point px_;
    geometry_msgs::Point py_;

    geometry_msgs::Point flx_;
    geometry_msgs::Point fly_;

    geometry_msgs::Point ffx_;
    geometry_msgs::Point ffy_;

    geometry_msgs::Point vc2_v_;  //(est des groundtruth)
    geometry_msgs::Point vc2_w_;   //(est des groundtruth)

    geometry_msgs::Point vc2_vx_;
    geometry_msgs::Point vc2_vy_;
    geometry_msgs::Point trig_;

    geometry_msgs::Point vc;

    ros::master::V_TopicInfo topic_list;
    bool getRosTopics(ros::master::V_TopicInfo& topics);
    bool autoscroll ;
    bool record;

    gazebo_msgs::ModelStates  model_state;
    gazebo_msgs::LinkStates  link_state;

};

#endif // MAINWINDOW_H
