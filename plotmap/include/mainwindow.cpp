#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <qptrajectory.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Timer = new QTimer(this);

    this->ros_sub_pub_init();
    this->plot_init();

    connect(Timer,SIGNAL(timeout()),this,SLOT(update()));
    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(click_button()));
    connect(ui->save_Button,SIGNAL(clicked()),this,SLOT(save_button()));
    connect(ui->qpsolve_Button,SIGNAL(clicked()),this,SLOT(qpsolve_button()));
    connect(ui->publish_button, SIGNAL(clicked()), this, SLOT(traj_pub_button()) );
    Timer->start(50);
}
void MainWindow::traj_pub_button(){
   //traj.time_from_start =0.02;
    trajetory_pub.publish(traj);
    ROS_WARN("trajectory has been published.");
}
void MainWindow::qpsolve_button2(){
    qp_curve_data.clear();
    std::vector<trajectory_profile> data;
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
    for(unsigned int i=0; i<1000; i++){
        double t = (1000-i)* (endtime/(1000-0));
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
        //std::cout << x << " \t " << y << std::endl;
        qp_curve_data.push_back(QCPCurveData(index,x,y));
        index++;

        double vx =  0*theta_x(0)+
                    1*theta_x(1)+
                    2*theta_x(2)*t+
                    3*theta_x(3)*t*t+
                    4*theta_x(4)*t*t*t+
                    5*theta_x(5)*t*t*t*t;
        double vy =  0*theta_y(0)+
                    1*theta_y(1)+
                    2*theta_y(2)*t+
                    3*theta_y(3)*t*t+
                    4*theta_y(4)*t*t*t+
                    5*theta_y(5)*t*t*t*t;
        ui->vel_plot->graph(0)->addData(i*endtime/(1000-0),vx);
        ui->vel_plot->graph(1)->addData(i*endtime/(1000-0),vy);
        ui->vel_plot->graph(0)->setPen(QPen(Qt::red));
    }
    std::cout << "index"<<  index  <<std::endl;
    path_curve->data()->clear();
    qp_path_curve->data()->set(qp_curve_data);
    QPen pen;
    pen.setWidth(3);
    pen.setColor(Qt::green);
    qp_path_curve->setPen(pen);
    ui->plot->replot();
    ui->vel_plot->replot();

}


void MainWindow::qpsolve_button(){
    std::vector<trajectory_profile> data;
    qp_curve_data.clear();


    //std::cout << "start   " << path_vec[0].b_c.pos.transpose() <<std::endl;
    if(path_vec.size()>0){
        data = plan.get_profile(path_vec, path_vec.size(), 0.05);
    }

    std::cout << "profile count" << data.size()<<std::endl;
    for(unsigned int i=30;i< (data.size()-10);i++){

        qp_curve_data.push_back(QCPCurveData(i,data[i].pos(0),data[i].pos(1)));

        ui->vel_plot->graph(0)->addData(i*0.02 , data[i].vel(0));
        ui->vel_plot->graph(1)->addData(i*0.02 , data[i].vel(1));
        //std::cout << data[i].pos.transpose()<<std::endl;

        geometry_msgs::Transform pos;
        geometry_msgs::Twist vel,acc;
        pos.translation.x = data[i].pos(0);
        pos.translation.y = data[i].pos(1);

        vel.linear.x = data[i].vel(0);
        vel.linear.y = data[i].vel(1);

        acc.linear.x = data[i].acc(0);
        acc.linear.y = data[i].acc(1);

        traj.transforms.push_back(pos);
        traj.velocities.push_back(vel);
        traj.accelerations.push_back(acc);
    }

    data.clear();
    path_curve->data()->clear();
    qp_path_curve->data()->set(qp_curve_data);
    QPen pen;
    pen.setWidth(3);
    pen.setColor(Qt::green);
    qp_path_curve->setPen(pen);
    ui->plot->replot();
}

void MainWindow::ros_sub_pub_init(){
    map_sub   = nh.subscribe("/map",10 , &MainWindow::map_cb,this );
    spath_sub = nh.subscribe("/sPath",10 , &MainWindow::spath_cb,this );
    start_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose" ,1 );
    end_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal" ,1 );
    trajetory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/obstacle_avoidance_path",1);
}
void MainWindow::plot_vehicle(){

    QCPItemLine   *line;
    line = new  QCPItemLine(ui->plot);

//    line->start->setCoords(path.poses[i+1].pose.position.x ,path.poses[i+1].pose.position.y);
//    line->end->setCoords( path.poses[i].pose.position.x ,path.poses[i].pose.position.y);

    line->setHead(QCPLineEnding::esSpikeArrow);

}
void MainWindow::plot_init(){

    ui->plot->setOpenGl(true,4);
    start_text  = new QCPItemText(ui->plot);
    origin_text = new QCPItemText(ui->plot);
    end_text = new QCPItemText(ui->plot);
    path_curve = new QCPCurve(ui->plot->xAxis , ui->plot->yAxis);
    qp_path_curve = new QCPCurve(ui->plot->xAxis,ui->plot->yAxis);
    ui->plot->setInteractions(QCP::iSelectAxes | QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot->axisRect()->setupFullAxesBox(true);
    ui->plot->xAxis->setTickLabels(false);
    ui->plot->yAxis->setTickLabels(false);
    ui->plot->xAxis->grid()->setVisible(false);
    ui->plot->yAxis->grid()->setVisible(false);



    ui->vel_plot->addGraph();
    ui->vel_plot->addGraph();



    ui->vel_plot->axisRect()->setupFullAxesBox(true);
    ui->vel_plot->setInteractions(QCP::iSelectAxes | QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui->vel_plot->xAxis->setRange(0,100);
    ui->vel_plot->yAxis->setRange(-3,3);

}
void MainWindow::save_button(){
    QString filename = QFileDialog::getSaveFileName(0,"Save file",QDir::currentPath(),
           "png files (*.png)",
               new QString("png files (*.png)"));
    ui->plot->savePng(filename, 0, 0, 3,100, -1);
}

void MainWindow::click_button(){

    geometry_msgs::PoseWithCovarianceStamped posec;
    posec.pose.pose.position.x = ui->start_x_text->text().toDouble();
    posec.pose.pose.position.y = ui->start_y_text->text().toDouble();
    posec.pose.pose.orientation.z =0.671442089592;
    posec.pose.pose.orientation.w =0.741057029064;
    posec.header.frame_id = "map";
    posec.header.stamp = ros::Time::now();
    start_pose_pub.publish(posec);

    geometry_msgs::PoseStamped  pose;
    pose.pose.position.x = ui->goal_x_text->text().toDouble();
    pose.pose.position.y = ui->goal_y_text->text().toDouble();
    pose.pose.orientation.z = 0.758135558017;
    pose.pose.orientation.w = 0.652096983332;

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    end_pose_pub.publish(pose);

}
void MainWindow::update(){
    if(!ros::ok())
    {
         QApplication::quit();
    }
    ros::spinOnce();
    ui->plot->replot();
}
void MainWindow::map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    map_grid_data = *msg;
    int cur_x=0;
    int cur_y=-1;
    std::cout << std::endl;
    std::cout << "the information of map" <<std::endl;
    std::cout << map_grid_data.info.width << " x " << map_grid_data.info.height<<std::endl;

    ui->plot->xAxis->setRange(0,map_grid_data.info.width);
    ui->plot->yAxis->setRange(0,map_grid_data.info.height);

    if(map_grid_data.data.size()>0){
        std::cout << "size" <<map_grid_data.data.size()  <<std::endl;

        for(unsigned int i=0;i< map_grid_data.data.size();i++){

            //split 1-d data to 2-D
            if( i% ( map_grid_data.info.width) ==0){
                cur_y+=1;
                cur_x=0;
            }else{
                cur_x+=1;
            }
            //the object is an obstacle if its value is not zero.
            if(map_grid_data.data[i] != 0){
               //plot all obstalces on the map.
                QCPItemRect* section = new QCPItemRect(ui->plot);
                section->topLeft->setType(QCPItemPosition::ptPlotCoords);
                section->topLeft->setAxes(ui->plot->xAxis, ui->plot->yAxis);
                section->bottomRight->setType(QCPItemPosition::ptPlotCoords);
                section->bottomRight->setAxes(ui->plot->xAxis, ui->plot->yAxis);
                section->topLeft->setCoords(cur_x*0.5-0.25 , cur_y*0.5+0.25);
                section->bottomRight->setCoords(cur_x*0.5 +0.25,cur_y*0.5-0.25);
                section->setBrush(QBrush(QColor(0,200,0,100)));
                section->setPen(Qt::NoPen);

            }
        }

    }

}
void MainWindow::spath_cb(const nav_msgs::Path::ConstPtr &msg){

    path = *msg;
    path_curve_data.clear();
    path_vec.clear();


    if(path.poses.size()>0){

        std::cout << "path data size:  "<<path.poses.size()<<std::endl;

        for(unsigned int i=0;i<path.poses.size();i++){
            path_curve_data.push_back(QCPCurveData(i , path.poses[i].pose.position.x , path.poses[i].pose.position.y));
            path_curve->data()->set(path_curve_data);
            QPen path_pen;
            path_pen.setWidth(3);

            path_pen.setColor(Qt::blue);

            path_curve->setPen(path_pen);

            if((i+1) <path.poses.size()  && (i%3==0) ){

            QCPItemLine   *line;
            line = new  QCPItemLine(ui->plot);

            line->start->setCoords(path.poses[i+1].pose.position.x ,path.poses[i+1].pose.position.y);
            line->end->setCoords( path.poses[i].pose.position.x ,path.poses[i].pose.position.y);

            line->setHead(QCPLineEnding::esSpikeArrow);



            QPen arrow_pen;
            arrow_pen.setWidth(2);
            arrow_pen.setColor(Qt::red);
            line->setPen(arrow_pen);
            line->setVisible(true);


            }


        }
        //inverse the waypoint vector  and select waypoinnts.
        std::vector<geometry_msgs::PoseStamped> tmp;
        tmp.clear();

        int  c = 0;
        geometry_msgs::PoseStamped last_pose;

        for(unsigned int i=0;i<path.poses.size();i++){
          tmp.push_back( path.poses[path.poses.size()-i-1]);
        }
//        for(unsigned int i=0;i< path.poses.size();i++){
//          //  geometry_msgs::PoseStamped cur_pose= path.poses[i];
//            double dist = (path.poses[path.poses.size()-i-1].pose.position.x - last_pose.pose.position.x)* (path.poses[path.poses.size()-i-1].pose.position.x - last_pose.pose.position.x)
//                          +(path.poses[path.poses.size()-i-1].pose.position.y - last_pose.pose.position.y)* (path.poses[path.poses.size()-i-1].pose.position.y - last_pose.pose.position.y);
//            dist = sqrt(dist);
////             ROS_WARN("%f",dist);
//            if(i==0){

//              tmp.push_back( path.poses[path.poses.size()-i-1]);
//              last_pose = path.poses[path.poses.size()-i-1];
//              //time_interval.push_back(0.002);
//            }else if(    (i>0)   &&    (dist>0.5 )              ){
//              tmp.push_back( path.poses[path.poses.size()-i-1]);
//              last_pose = path.poses[path.poses.size()-i-1];
//              //time_interval.push_back(0.005);

//            }


//        }

        time_interval.clear();
        for(unsigned int k=0;k<tmp.size();k++){
            if( (k< tmp.size()-1)  ){
                trajectory_profile start, end;
                start.pos << tmp[k].pose.position.x,tmp[k].pose.position.y,0;
                end.pos << tmp[k+1].pose.position.x,tmp[k+1].pose.position.y,0;
                double dist = (start.pos - end.pos).norm();
                ROS_WARN(" dist = %f",dist);
                time_interval.push_back(dist);
                path_vec.push_back(segments(start,end,dist*3));
            }
        }

        //marker  location of start and goal
        int end_id = 0;
        int start_id= path.poses.size()-1;

        start_text->position->setCoords(path.poses[start_id].pose.position.x, path.poses[start_id].pose.position.y); // move 10 pixels to the top from bracket center anchor
        start_text->setPositionAlignment(Qt::AlignHCenter);
        start_text->setText("Start");
        start_text->setFont(QFont(font().family(), 20));


        origin_text->position->setCoords(0,0);
        origin_text->setPositionAlignment(Qt::AlignHCenter);
        origin_text->setText("Origin");
        origin_text->setFont(QFont(font().family(), 20));

        end_text->position->setCoords(path.poses[end_id].pose.position.x-1 , path.poses[end_id].pose.position.y);
        end_text->setPositionAlignment(Qt::AlignHCenter|Qt::AlignLeft);
        end_text->setText("End");
        end_text->setFont(QFont(font().family(), 20));

        std::cout << "path_vec size : " <<path_vec.size() <<std::endl;

    }
}
MainWindow::~MainWindow()
{

    delete ui;
}
