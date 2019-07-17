#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qcustomplot.h>
#include <ros/ros.h>

#include <QTimer>
#include <QTime>
//#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <qcustomplot.h>
#include <QString>
#include <qptrajectory.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    ros::NodeHandle nh;
    ros::Publisher start_pose_pub;
    ros::Publisher end_pose_pub;
    ros::Publisher trajetory_pub;
   // ros::Publisher traj_pub;


    ros::Subscriber map_sub;
    ros::Subscriber spath_sub;
    QTimer *Timer;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    nav_msgs::OccupancyGrid map_grid_data;

private:

    void ros_sub_pub_init();
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void spath_cb(const nav_msgs::Path::ConstPtr &msg);
    void plot_init();
    void plot_vehicle();
    Ui::MainWindow *ui;
    QCPItemText *start_text;
    QCPItemText *end_text;
    QCPItemText *origin_text;

    std::vector<double>   time_interval;
    QCPCurve *qp_path_curve;
    QVector<QCPCurveData> qp_curve_data;

    QCPCurve *path_curve;
    QVector<QCPCurveData> path_curve_data;

    qptrajectory plan;
    path_def path_vec;
    nav_msgs::Path path;
    nav_msgs::Path obstacle_path;
    trajectory_msgs::MultiDOFJointTrajectoryPoint traj;
private slots:
    void update();
    void click_button();
    void save_button();
    void qpsolve_button();
    void qpsolve_button2();
    void traj_pub_button();
};

#endif // MAINWINDOW_H
