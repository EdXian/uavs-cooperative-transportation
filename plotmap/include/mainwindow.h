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
//#include < geometry_msgs/PoseWithCovarianceStamped.h>
#include <qcustomplot.h>
#include <QString>
#include <qptrajectory.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
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

    ros::Subscriber map_sub;
    ros::Subscriber spath_sub;
    ros::Publisher traj_pub;
    QTimer *Timer;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    nav_msgs::OccupancyGrid map_grid_data;

private:

    void ros_sub_pub_init();
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void spath_cb(const nav_msgs::Path::ConstPtr &msg);
    void plot_init();
    Ui::MainWindow *ui;
    QCPItemText *start_text;
    QCPItemText *end_text;

    QCPCurve *qp_path_curve;
    QVector<QCPCurveData> qp_curve_data;

    QCPCurve *path_curve;
    QVector<QCPCurveData> path_curve_data;

    qptrajectory plan;
    path_def path_vec;
      nav_msgs::Path path;
    nav_msgs::Path obstacle_path;
private slots:
    void update();
    void click_button();
    void save_button();
    void qpsolve_button();
    void qpsolve_button2();
};

#endif // MAINWINDOW_H
