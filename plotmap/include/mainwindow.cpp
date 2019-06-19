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
    Timer->start(50);

}
void MainWindow::qpsolve_button2(){

}
void MainWindow::qpsolve_button(){
    qp_curve_data.clear();
    std::vector<trajectory_profile> data;
    double dt=0.1;
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
//        std::cout << tmp_path.poses[i].pose.position.x << "  "
//                << tmp_path.poses[i].pose.position.y<<std::endl;
    }
    path = tmp_path;
    //split the path into 10 segments.
    for(unsigned int i=0; i< (path.poses.size()/seg_size)+1 ; i++){
        nav_msgs::Path data;
        if((path.poses.size() - count)/seg_size > 0){
            for(unsigned int j=0;j<10;j++){
                data.poses.push_back(path.poses[count]);
                count++;
            }
        }else{
            int redun = path.poses.size();
            for(unsigned int j=0;j<(redun - (redun/10)*10 );j++){
                data.poses.push_back(path.poses[count]);
                count++;
            }

        }
        seg_vec.push_back(data);
    }

//    std::cout <<  "segments size = "   <<seg_vec.size()<<std::endl;
//    for(unsigned int i=0;i<seg_vec.size();i++){
//        std::cout <<  "seg" << i << std::endl;
//        for(unsigned int j=0;j<seg_vec[i].poses.size();j++){
//            std::cout << seg_vec[i].poses[j].pose.position.x <<"  "
//                      << seg_vec[i].poses[j].pose.position.y << std::endl;

//        }
//    }



    for(unsigned int m=0;m<seg_vec.size();m++){
            A_x.setZero(seg_vec[m].poses.size(),6); //sixth order polynomial
            A_y.setZero(seg_vec[m].poses.size(),6);
            C_x.setZero(seg_vec[m].poses.size());
            C_y.setZero(seg_vec[m].poses.size());
            T=0;
            for(unsigned int i=0;i<seg_vec[m].poses.size();i++){
                double t_k=1;
                for(unsigned int j=0;j<6;j++){
                    A_x(i,j) = t_k;
                    t_k = t_k* T;
                }
                T+=dt;
            }

            A_y = A_x;
            for(unsigned int j=0;j<seg_vec[m].poses.size();j++){
                C_x(j) = seg_vec[m].poses[seg_vec[m].poses.size()-1-j].pose.position.x;
                C_y(j) = seg_vec[m].poses[seg_vec[m].poses.size()-1-j].pose.position.y;
            }
            theta_x = (A_x.transpose()*A_x).inverse()*A_x.transpose()*C_x;
            theta_y = (A_y.transpose()*A_y).inverse()*A_y.transpose()*C_y;
            v_x.push_back(theta_x);
            v_y.push_back(theta_y);
    }
//    std::cout << v_x.size() << "  " <<v_y.size()<<std::endl;
//    for(unsigned int i=0;i<v_x.size();i++){
//        std::cout << "==================="<<std::endl;
//        std::cout << v_x[i].transpose() << std::endl << v_y[i].transpose() <<std::endl;
//    }
    std::vector<double> px,py;
    for(unsigned int i=0;i<v_x.size()-1;i++){
        for(unsigned int j=0;j<10;j++){
            double t = (10-j)*dt;

            double x =  v_x[i](0)+
                        v_x[i](1)*t+
                        v_x[i](2)*t*t+
                        v_x[i](3)*t*t*t+
                        v_x[i](4)*t*t*t*t+
                        v_x[i](5)*t*t*t*t*t;
            double y =  v_y[i](0)+
                        v_y[i](1)*t+
                        v_y[i](2)*t*t+
                        v_y[i](3)*t*t*t+
                        v_y[i](4)*t*t*t*t+
                        v_y[i](5)*t*t*t*t*t;
                        px.push_back(x) ;
                        py.push_back(y);
//           qp_curve_data.push_back(QCPCurveData(index,x,y));
//            index++;
        }
    }


    for(unsigned int i=0;i<px.size();i++){
        std::cout << px[i] << " ==  " << py[i] << std::endl;
        qp_curve_data.push_back(QCPCurveData(i,px[i],py[i]));

    }
    path_curve->data()->clear();
    qp_path_curve->data()->set(qp_curve_data);
    QPen pen;
    pen.setWidth(3);
    pen.setColor(Qt::green);
    qp_path_curve->setPen(pen);
    ui->plot->replot();

   // traj_pub.publish(obstacle_path);
  //  std::cout << " profile counts : " << data.size() << std::endl;
}

void MainWindow::ros_sub_pub_init(){
    map_sub   = nh.subscribe("/map",10 , &MainWindow::map_cb,this );
    spath_sub = nh.subscribe("/sPath",10 , &MainWindow::spath_cb,this );
    start_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose" ,1 );
    end_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal" ,1 );
    traj_pub = nh.advertise<nav_msgs::Path>("/obstacle_avoidance_path",1);
}

void MainWindow::plot_init(){

    ui->plot->setOpenGl(true,4);
    start_text  = new QCPItemText(ui->plot);
    end_text = new QCPItemText(ui->plot);
    path_curve = new QCPCurve(ui->plot->xAxis , ui->plot->yAxis);
    qp_path_curve = new QCPCurve(ui->plot->xAxis,ui->plot->yAxis);
    ui->plot->setInteractions(QCP::iSelectAxes | QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot->axisRect()->setupFullAxesBox(true);
    ui->plot->xAxis->setTickLabels(false);
    ui->plot->yAxis->setTickLabels(false);
    ui->plot->xAxis->grid()->setVisible(false);
    ui->plot->yAxis->grid()->setVisible(false);


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
                cur_y++;
                cur_x=0;
            }else{
                cur_x++;
            }
            //the object is an obstacle if its value is not zero.
            if(map_grid_data.data[i] != 0){
               //plot all obstalces on the map.
                QCPItemRect* section = new QCPItemRect(ui->plot);
                section->topLeft->setType(QCPItemPosition::ptPlotCoords);
                section->topLeft->setAxes(ui->plot->xAxis, ui->plot->yAxis);
                section->bottomRight->setType(QCPItemPosition::ptPlotCoords);
                section->bottomRight->setAxes(ui->plot->xAxis, ui->plot->yAxis);
                section->topLeft->setCoords(cur_x-0.5 , cur_y+0.5);
                section->bottomRight->setCoords(cur_x +0.5,cur_y-0.5);
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

//            if( (i< path.poses.size()-1)){
//                trajectory_profile start, end;
//                start.pos << path.poses[i+1].pose.position.x,path.poses[i+1].pose.position.y,0;
//                end.pos << path.poses[i].pose.position.x,path.poses[i].pose.position.y,0;
//                path_vec.push_back(segments(start,end,1.0));
//            }

            QPen arrow_pen;
            arrow_pen.setWidth(2);
            arrow_pen.setColor(Qt::red);
            line->setPen(arrow_pen);
            line->setVisible(true);


            }


        }
        //inverse the waypoint vector
        std::vector<geometry_msgs::PoseStamped> tmp;
        tmp.clear();
        tmp.resize(path.poses.size());
        for(unsigned int i=0;i< path.poses.size();i++){
           tmp[i] = path.poses[path.poses.size()-i-1];
        }
//        for( unsigned int i=0;i< tmp.size();i++){
//            if( (i< (tmp.size()-1))){
//                if(i%10 == 0){
//                    trajectory_profile start, end;
//                    start.pos << tmp[i].pose.position.x,tmp[i].pose.position.y,0;
//                    end.pos << tmp[i+10].pose.position.x,tmp[i+10].pose.position.y,0;
//                    std::cout << i<< "  " <<start.pos.transpose() << "\t "<< end.pos.transpose()<<std::endl;
//                    path_vec.push_back(segments(start,end,1.0));

//                }
//            }
//        }

        //marker  location of start and goal
        int end_id = 0;
        int start_id= path.poses.size()-1;

        start_text->position->setCoords(path.poses[start_id].pose.position.x, path.poses[start_id].pose.position.y); // move 10 pixels to the top from bracket center anchor
        start_text->setPositionAlignment(Qt::AlignHCenter);
        start_text->setText("Start");
        start_text->setFont(QFont(font().family(), 20));

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
