
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "tf/transform_datatypes.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    //curve_data(100),
    autoscroll(true),
    record(true)
{
    ui->setupUi(this);
    this->setWindowTitle("cooperative transportation");
    rotation_matrix.setZero(3,3);
   // local_sub = nh.subscribe<geometry_msgs::Point>("/uav2/local_position",2,&MainWindow::local_cb,this);
   // traj_sub=nh.subscribe<geometry_msgs::Point>("/uav2/desired_position",2,&MainWindow::traj_cb,this);
    desired_traj_sub =nh.subscribe<geometry_msgs::Point>("/drone1/desired_position",2 ,&MainWindow::desired_traj_cb,this);

    desired_force_sub = nh.subscribe<geometry_msgs::Point>("/desired_force",2,&MainWindow::desired_force_cb,this);
    desired_vel_sub = nh.subscribe<geometry_msgs::Point>("/desired_velocity",2,&MainWindow::desired_velocity_cb,this);

    estimate_force_sub=nh.subscribe<geometry_msgs::Point>("/follower_ukf/force_estimate",2,&MainWindow::estimate_force_cb,this);
    trigger_sub=nh.subscribe<geometry_msgs::Point>("/follower_trigger",2,&MainWindow::trigger_cb,this);

    vel_est_sub = nh.subscribe("est_vel", 2,&MainWindow::vel_est_cb,this);
    vel_estb_sub = nh.subscribe("pointvc2", 2,&MainWindow::vel_est_b_cb,this);
    model_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",2,&MainWindow::model_cb,this);
    link_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",2,&MainWindow::link_cb,this);
    force2_sub= nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor2_topic",2,&MainWindow::force2_cb,this);
    force1_sub  = nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor1_topic",2 , &MainWindow::force1_cb,this);
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",1,&MainWindow::map_cb,this);

    px=nh.advertise<geometry_msgs::Point>("/data/px",2);
    py=nh.advertise<geometry_msgs::Point>("/data/py",2);

    vcb = nh.advertise<geometry_msgs::Point>("/data/vc",2);

    flx = nh.advertise<geometry_msgs::Point>("/data/flx",2);
    fly = nh.advertise<geometry_msgs::Point>("/data/fly",2);

    ffx = nh.advertise<geometry_msgs::Point>("/data/ffx",2);
    ffy = nh.advertise<geometry_msgs::Point>("/data/ffy",2);

    vc2_v = nh.advertise<geometry_msgs::Point>("/data/vc2_vel",2);
    vc2_w=nh.advertise<geometry_msgs::Point>("/data/vc2_omega",2);

    vc2_vx = nh.advertise<geometry_msgs::Point>("/data/vc2_vx",2);
    vc2_vy = nh.advertise<geometry_msgs::Point>("/data/vc2_vy",2);
    trig = nh.advertise<geometry_msgs::Point>("/data/trigger",2);


    conn1_curve = new QCPCurve(ui->customplot4->xAxis ,ui->customplot4->yAxis );
    conn2_curve = new QCPCurve(ui->customplot4->xAxis ,ui->customplot4->yAxis );
    conn1_curve->data()->set( conn1_curve_data, true);
    conn2_curve->data()->set( conn2_curve_data, true);


    conn1_curve->setName("conn1");
    conn2_curve->setName("conn2");

    ui->customplot->setAntialiasedElement(QCP::aeAll , true);
    path_curve =new QCPCurve(ui->customplot4->xAxis,ui->customplot4->yAxis);
    path_curve_data.clear();
    path_curve->data()->set(path_curve_data, true);
    path_curve->setName("Desired Trajectory \n (point c2)");

    map_path_curve =new QCPCurve(ui->plot->xAxis,ui->plot->yAxis);
    map_path_curve_data.clear();
    map_path_curve->data()->set(map_path_curve_data, true);
    map_path_curve->setName("Desired Trajectory \n (point c2)");

    desired_curve = new QCPCurve(ui->customplot4->xAxis, ui->customplot4->yAxis);
    desired_curve_data.clear();
    desired_curve->data()->set(desired_curve_data, true);
    desired_curve->setName("Actual Trajectory \n (point c2)");

    map_desired_curve = new QCPCurve(ui->plot->xAxis, ui->plot->yAxis);
    map_desired_curve_data.clear();
    map_desired_curve->data()->set(map_desired_curve_data, true);
    map_desired_curve->setName("Actual Trajectory \n (point c2)");


    QFont legend_font;
    legend_font.setFamily("System");
    legend_font.setPointSize(17);

    ui->customplot->setOpenGl(true,8);

    ui->customplot->setInteractions(QCP::iSelectAxes | QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    QFont font;
    font.setStyleHint(QFont::System);
    QPen pen;
    pen.setWidth(2);
    font.setPointSize(17);

    ui->customplot->legend->setVisible(true);
    ui->customplot->xAxis->setLabelFont(font);
    ui->customplot->xAxis->setTickLabelFont(font);
    ui->customplot->xAxis->setLabel("Time (sec)");
    ui->customplot->yAxis->setLabel("Displacement (m)");
    ui->customplot->axisRect()->setupFullAxesBox(true);


    ui->customplot->addGraph();
    ui->customplot->graph(0)->setName(QString("Ground truth" ));
    pen.setColor(Qt::gray);
    pen.setWidth(2);
    ui->customplot->graph(0)->setPen(pen);

    ui->customplot->addGraph();
    pen.setWidth(2);
    ui->customplot->graph(1)->setName(QString("Desired x"));
    pen.setColor(Qt::green);
    ui->customplot->graph(1)->setPen(pen);

    ui->customplot->addGraph();
    pen.setWidth(2);
    ui->customplot->graph(2)->setName(QString("error"));
    pen.setColor(Qt::red);
    ui->customplot->graph(2)->setPen(pen);



    pen.setWidth(2);
    //legend_font.setPointSize(13);
    ui->customplot->legend->setFont(legend_font);

    pen.setColor(Qt::green);
    conn1_curve->setPen(pen);


    pen.setColor(Qt:: black);
    conn2_curve->setPen(pen);
    conn1_curve->setVisible(false);
    conn1_curve->setName("leader trajectory");
    conn1_curve->removeFromLegend();

    conn2_curve->setVisible(false);
    conn2_curve->removeFromLegend();

    QFont tickfont;
    tickfont.setPointSize(17);
    ui->customplot->yAxis->setTickLabelFont(tickfont);
    ui->customplot->yAxis->setLabelFont(tickfont);

    ui->customplot2->setInteractions(QCP::iSelectAxes | QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    font.setStyleHint(QFont::System);
    font.setPointSize(17);
    ui->customplot2->legend->setVisible(true);

    ui->customplot2->axisRect()->setupFullAxesBox(true);
    ui->customplot2->xAxis->setLabelFont(font);
    ui->customplot2->xAxis->setTickLabelFont(font);
    ui->customplot2->yAxis->setLabelFont(font);
    ui->customplot2->yAxis->setTickLabelFont(font);
    ui->customplot2->xAxis->setLabel("Time (sec)");
    ui->customplot2->yAxis->setLabel("Displacement (m)");
    QFont afont;
    //afont.setPointSize(67);
    afont.setPointSize(1);

    legend_font.setFamily("system");
    //legend_font.setPointSize(13);
    ui->customplot2->legend->setFont(legend_font);

    ui->customplot2->addGraph();
    ui->customplot2->graph(0)->setName(QString("Ground truth"));
    ui->customplot2->addGraph();
    ui->customplot2->graph(1)->setName(QString("Desired y"));

    ui->customplot2->addGraph();
    ui->customplot2->graph(2)->setName(QString("error"));



    ui->customplot3->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
    font.setStyleHint(QFont::System);
    font.setPointSize(17);
    ui->customplot3->legend->setVisible(true);
//    ui->customplot3->plotLayout()->insertRow(0);
//    QCPTextElement *title3 = new QCPTextElement(ui->customplot3, "X (m)", QFont("system", 17, QFont::Bold));
//    ui->customplot3->plotLayout()->addElement(0, 0, title3);
    ui->customplot3->axisRect()->setupFullAxesBox(true);
    ui->customplot3->xAxis->setLabelFont(font);
    ui->customplot3->xAxis->setTickLabelFont(font);
    ui->customplot3->yAxis->setLabelFont(font);
    ui->customplot3->yAxis->setTickLabelFont(font);
    ui->customplot3->xAxis->setLabel("X (m)");
    ui->customplot3->yAxis->setLabel("");

    legend_font.setFamily("system");
    ui->customplot3->legend->setFont(legend_font);

    ui->customplot3->addGraph();
    ui->customplot3->graph(0)->setName(QString("Estimate z"));

    ui->customplot3->addGraph();
    ui->customplot3->graph(1)->setName(QString("Desired z"));

    QFont tfont;
    tfont.setPointSize(17);
    ui->customplot3->yAxis->setTickLabelFont(tfont);

    pen.setColor(Qt::blue);
    ui->customplot2->graph(0)->setPen(pen);

    pen.setColor(Qt::green);
    ui->customplot2->graph(1)->setPen(pen);

    pen.setColor(Qt::red);
    ui->customplot2->graph(2)->setPen(pen);


    ui->customplot4->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);

    ui->customplot4->xAxis->setLabel("X (m)");

    ui->customplot4->yAxis->setLabel("Y (m)");

    ui->customplot4->xAxis->setRange(-10,10);
    ui->customplot4->yAxis->setRange(-10,10);



    ui->qcustomplot->addGraph();
    ui->qcustomplot2->addGraph();
    ui->qcustomplot->addGraph();
    ui->qcustomplot2->addGraph();
    ui->qcustomplot->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
    ui->qcustomplot2->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
    ui->qcustomplot->graph(0)->setName(QString("force x"));
    ui->qcustomplot->graph(1)->setName(QString("desired_force x"));
    ui->qcustomplot2->graph(0)->setName(QString("force y"));
    ui->qcustomplot2->graph(1)->setName(QString("desired_force y"));
    ui->qcustomplot->legend->setVisible(true);
    ui->qcustomplot2->legend->setVisible(true);

     ui->qcustomplot2->xAxis->setLabel("time");
     ui->qcustomplot2->yAxis->setLabel("N");

     ui->qcustomplot->xAxis->setLabel("time");
     ui->qcustomplot->yAxis->setLabel("N");
     ui->qcustomplot->xAxis->setTickLabelFont(tfont);
     ui->qcustomplot->yAxis->setTickLabelFont(tfont);
     ui->qcustomplot2->xAxis->setTickLabelFont(tfont);
     ui->qcustomplot2->yAxis->setTickLabelFont(tfont);

     ui->qcustomplot->xAxis->setLabelFont(tfont);
     ui->qcustomplot->yAxis->setLabelFont(tfont);
     ui->qcustomplot2->xAxis->setLabelFont(tfont);
     ui->qcustomplot2->yAxis->setLabelFont(tfont);
     ui->qcustomplot->legend->setFont(tfont);
     ui->qcustomplot2->legend->setFont(tfont);


     ui->qcustomplot3->addGraph();
     ui->qcustomplot4->addGraph();
     ui->qcustomplot3->addGraph();
     ui->qcustomplot4->addGraph();
     ui->qcustomplot3->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
     ui->qcustomplot4->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
     ui->qcustomplot3->graph(0)->setName(QString("force x \n payload frame"));
     ui->qcustomplot3->graph(1)->setName(QString("Estimation force x"));
     ui->qcustomplot4->graph(0)->setName(QString("force y \n payload frame"));
     ui->qcustomplot4->graph(1)->setName(QString("Estimation force y"));

//    ui->qcustomplot4->graph(1)->setVisible(false);
//     ui->qcustomplot3->graph(1)->setVisible(false);
//    ui->qcustomplot3->graph(1)->removeFromLegend();
//    ui->qcustomplot4->graph(1)->removeFromLegend();
     ui->plot->setInteractions(QCP::iSelectAxes | QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
     ui->plot->axisRect()->setupFullAxesBox(true);
     ui->plot->xAxis->setTickLabels(false);
     ui->plot->yAxis->setTickLabels(false);
     ui->plot->xAxis->grid()->setVisible(false);
     ui->plot->yAxis->grid()->setVisible(false);



     ui->customplot4->legend->setVisible(true);
     ui->customplot4->legend->setFont(tfont);
     ui->customplot4->xAxis->setTickLabelFont(tfont);
     ui->customplot4->yAxis->setTickLabelFont(tfont);
     ui->customplot4->yAxis->setLabelFont(tfont);
     ui->customplot4->xAxis->setLabelFont(tfont);



     ui->qcustomplot3->legend->setVisible(true);
     ui->qcustomplot4->legend->setVisible(true);

      ui->qcustomplot4->xAxis->setLabel("time");
      ui->qcustomplot4->yAxis->setLabel("N");

      ui->qcustomplot3->xAxis->setLabel("time");
      ui->qcustomplot3->yAxis->setLabel("N");
      ui->qcustomplot3->xAxis->setTickLabelFont(tfont);
      ui->qcustomplot3->yAxis->setTickLabelFont(tfont);
      ui->qcustomplot4->xAxis->setTickLabelFont(tfont);
      ui->qcustomplot4->yAxis->setTickLabelFont(tfont);

      ui->qcustomplot3->xAxis->setLabelFont(tfont);
      ui->qcustomplot3->yAxis->setLabelFont(tfont);
      ui->qcustomplot4->xAxis->setLabelFont(tfont);
      ui->qcustomplot4->yAxis->setLabelFont(tfont);
      ui->qcustomplot3->legend->setFont(tfont);
      ui->qcustomplot4->legend->setFont(tfont);

      ui->qcustomplot3->xAxis->setRange(0,125);
      ui->qcustomplot4->xAxis->setRange(0,125);

      ui->qcustomplot3->yAxis->setRange(-3,3);
      ui->qcustomplot4->yAxis->setRange(-3,3);


      ui->qcustomplot5->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot5->addGraph();
      ui->qcustomplot5->addGraph();


      ui->qcustomplot6->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot6->addGraph();
      ui->qcustomplot6->addGraph();
      ui->qcustomplot7->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot7->addGraph();
      ui->qcustomplot7->addGraph();

      ui->qcustomplot8->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot8->addGraph();
      ui->qcustomplot8->addGraph();
      ui->qcustomplot9->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot9->addGraph();
      ui->qcustomplot9->addGraph();

      ui->qcustomplot8->graph(0)->setName("Estimated velocity");
      ui->qcustomplot9->graph(0)->setName("Estimated omega");
      ui->qcustomplot8->graph(1)->setName("ground truth ");
      ui->qcustomplot9->graph(1)->setName("ground truth");
      ui->qcustomplot8->legend->setVisible(true);
      ui->qcustomplot9->legend->setVisible(true);

      ui->qcustomplot8->xAxis->setRange(0,125);
      ui->qcustomplot9->xAxis->setRange(0,125);
      ui->qcustomplot8->yAxis->setRange(-5,5);
      ui->qcustomplot9->yAxis->setRange(-5,5);
      ui->qcustomplot8->xAxis->setTickLabelFont(tfont);
      ui->qcustomplot9->xAxis->setTickLabelFont(tfont);
      ui->qcustomplot8->yAxis->setTickLabelFont(tfont);
      ui->qcustomplot9->yAxis->setTickLabelFont(tfont);
      ui->qcustomplot8->xAxis->setLabel("time");
      ui->qcustomplot8->yAxis->setLabel("Vel (m/s)");
      ui->qcustomplot9->xAxis->setLabel("time");
      ui->qcustomplot9->yAxis->setLabel("Vel (rad/s)");
      ui->qcustomplot9->yAxis->setLabelFont(tfont);
      ui->qcustomplot9->xAxis->setLabelFont(tfont);
      ui->qcustomplot8->yAxis->setLabelFont(tfont);
      ui->qcustomplot8->xAxis->setLabelFont(tfont);
      ui->qcustomplot8->axisRect()->setupFullAxesBox(true);
      ui->qcustomplot9->axisRect()->setupFullAxesBox(true);
      ui->qcustomplot8->legend->setFont(tfont);
      ui->qcustomplot9->legend->setFont(tfont);


      ui->qcustomplot10->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot10->addGraph();

      ui->qcustomplot11->setInteractions(QCP::iRangeZoom | QCP::iSelectAxes | QCP::iRangeDrag  | QCP::iSelectPlottables);
      ui->qcustomplot11->addGraph();


      ui->qcustomplot10->graph(0)->setName("vc2_x \n(payload frame)");
      ui->qcustomplot11->graph(0)->setName("vc2_y \n(payload frame)");
      ui->qcustomplot10->legend->setVisible(true);
      ui->qcustomplot11->legend->setVisible(true);

      ui->qcustomplot10->xAxis->setRange(0,125);
      ui->qcustomplot11->xAxis->setRange(0,125);
      ui->qcustomplot10->yAxis->setRange(-5,5);
      ui->qcustomplot11->yAxis->setRange(-5,5);
      ui->qcustomplot10->xAxis->setTickLabelFont(tfont);
      ui->qcustomplot11->xAxis->setTickLabelFont(tfont);
      ui->qcustomplot10->yAxis->setTickLabelFont(tfont);
      ui->qcustomplot11->yAxis->setTickLabelFont(tfont);
      ui->qcustomplot10->xAxis->setLabel("time");
      ui->qcustomplot10->yAxis->setLabel("Vel (m/s)");
      ui->qcustomplot11->xAxis->setLabel("time");
      ui->qcustomplot11->yAxis->setLabel("Vel (rad/s)");
      ui->qcustomplot11->yAxis->setLabelFont(tfont);
      ui->qcustomplot11->xAxis->setLabelFont(tfont);
      ui->qcustomplot10->yAxis->setLabelFont(tfont);
      ui->qcustomplot10->xAxis->setLabelFont(tfont);
      ui->qcustomplot10->axisRect()->setupFullAxesBox(true);
      ui->qcustomplot11->axisRect()->setupFullAxesBox(true);
      ui->qcustomplot10->legend->setFont(tfont);
      ui->qcustomplot11->legend->setFont(tfont);


    ui->qcustomplot5->graph(0)->setName(QString("Estimate total force "));
    ui->qcustomplot5->graph(1)->setName(QString("trigger"));

    ui->qcustomplot6->graph(0)->setName("desired vel \n(payload)");
    ui->qcustomplot7->graph(0)->setName("desired omega \n(payload)");
    ui->qcustomplot6->graph(1)->setName("estimated vel ");
    ui->qcustomplot7->graph(1)->setName("estimated omega");
    ui->qcustomplot6->legend->setVisible(true);
    ui->qcustomplot7->legend->setVisible(true);

    ui->qcustomplot6->xAxis->setRange(0,125);
    ui->qcustomplot7->xAxis->setRange(0,125);
    ui->qcustomplot6->yAxis->setRange(-5,5);
    ui->qcustomplot7->yAxis->setRange(-5,5);
    ui->qcustomplot6->xAxis->setTickLabelFont(tfont);
    ui->qcustomplot7->xAxis->setTickLabelFont(tfont);
    ui->qcustomplot6->yAxis->setTickLabelFont(tfont);
    ui->qcustomplot7->yAxis->setTickLabelFont(tfont);
    ui->qcustomplot6->xAxis->setLabel("time");
    ui->qcustomplot6->yAxis->setLabel("Vel (m/s)");
    ui->qcustomplot7->xAxis->setLabel("time");
    ui->qcustomplot7->yAxis->setLabel("Omega (rad/s)");
    ui->qcustomplot7->yAxis->setLabelFont(tfont);
    ui->qcustomplot7->xAxis->setLabelFont(tfont);
    ui->qcustomplot6->yAxis->setLabelFont(tfont);
    ui->qcustomplot6->xAxis->setLabelFont(tfont);

    ui->qcustomplot6->axisRect()->setupFullAxesBox(true);
    ui->qcustomplot7->axisRect()->setupFullAxesBox(true);
    ui->qcustomplot6->legend->setFont(tfont);
    ui->qcustomplot7->legend->setFont(tfont);
    ui->qcustomplot5->xAxis->setRange(0,125);

    ui->qcustomplot5->yAxis->setRange(-3,3);
    ui->qcustomplot5->legend->setVisible(true);
    ui->qcustomplot5->legend->setVisible(true);
    ui->qcustomplot5->xAxis->setTickLabelFont(tfont);
    ui->qcustomplot5->yAxis->setTickLabelFont(tfont);

    ui->qcustomplot5->xAxis->setLabel("time");
    ui->qcustomplot5->yAxis->setLabel("N");
    ui->qcustomplot5->xAxis->setLabelFont(tfont);
    ui->qcustomplot5->yAxis->setLabelFont(tfont);
    ui->qcustomplot5->legend->setFont(tfont);
    ui->qcustomplot5->legend->setFont(tfont);

    ui->qcustomplot->axisRect()->setupFullAxesBox(true);
    ui->qcustomplot2->axisRect()->setupFullAxesBox(true);
    ui->qcustomplot3->axisRect()->setupFullAxesBox(true);
    ui->qcustomplot4->axisRect()->setupFullAxesBox(true);
    ui->qcustomplot5->axisRect()->setupFullAxesBox(true);

    ui->customplot4->axisRect()->setupFullAxesBox(true);

    pen.setColor(Qt::blue);
    ui->qcustomplot5->graph(0)->setPen(pen);

    pen.setColor(Qt::green);
    ui->qcustomplot5->graph(1)->setPen(pen);

    QPen tpen;
    tpen =pen;

    pen.setColor(Qt::darkBlue);
    pen.setStyle(Qt::PenStyle::DashLine);
    pen.setWidth(4);
    path_curve->setPen(pen);
    map_path_curve->setPen(pen);
    pen = tpen;



    pen.setColor(Qt::red);
    desired_curve->setPen(pen);
    map_desired_curve->setPen(pen);
    pen.setColor(Qt::red);

    ui->customplot2->yAxis->setRange(-3,4);
    ui->customplot->yAxis->setRange(-5,5);

    ui->customplot3->yAxis->setRange(-10,10);
    ui->customplot3->xAxis->setRange(-10,10);

    ui->customplot2->xAxis->setRange(0,65);
    ui->customplot->xAxis->setRange(0,65);
    ui->customplot3->yAxis->setLabel("Y (m)");

    ui->qcustomplot->yAxis->setRange(-4,4);
    ui->qcustomplot2->yAxis->setRange(-4,4);

    ui->qcustomplot->xAxis->setRange(0,125);
    ui->qcustomplot2->xAxis->setRange(0,125);


    ui->customplot->replot();
    ui->customplot2->replot();
    ui->customplot3->replot();


    ui->maxspinbox->setMaximum(1000.0);
    ui->minspinbox->setMinimum(0.0);

    state_timer = new QTimer(this);
    connect(state_timer, SIGNAL(timeout()),this, SLOT(state_loop()));
    state_timer->start(100);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this, SLOT(plot_loop()));
    timer->start(100);
}
void MainWindow::vel_est_b_cb(const geometry_msgs::Point::ConstPtr &msg){
   c2_vel_b = *msg;


}


void MainWindow::vel_est_cb(const  geometry_msgs::Point::ConstPtr& msg){
    c2_vel = *msg;
}
MainWindow::~MainWindow()
{
  delete ui;
  delete timer;
  delete state_timer;

}
void MainWindow::state_loop(){
  ros::spinOnce();


}

void MainWindow::map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    map_grid_data = *msg;
    int cur_x=0;
    int cur_y=-1;

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
                section->setBrush(QBrush(QColor(0,0,0,100)));
                section->setPen(Qt::NoPen);

            }
        }

    }

}





void MainWindow::force2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    QPen pen;
    pen.setColor(Qt::blue);
    pen.setWidth(2);
    Eigen::Vector3d data , adata;
    data <<msg->wrench.force.x , msg->wrench.force.y ,msg->wrench.force.z;


    adata = payload_link1_rotation*data;
    data = rotation_matrix * adata;
    ui->qcustomplot3->graph(0)->addData(time , data(0));
    ui->qcustomplot4->graph(0)->addData(time ,data(1));

    ffx_.x = data(0);
    ffy_.x = data(1);

    ui->qcustomplot3->graph(0)->setPen(pen);
    ui->qcustomplot4->graph(0)->setPen(pen);
}
void MainWindow::estimate_force_cb(const geometry_msgs::Point::ConstPtr& msg){
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(2);

    Eigen::Vector3d f,f_;

    f<<msg->x,msg->y,msg->z;
    f_=rotation_matrix*f ;



    ui->qcustomplot3->graph(1)->addData(time , -1.0*f_(0));
    ui->qcustomplot4->graph(1)->addData(time , -1.0*f_(1));

    ffx_.y = -1.0*f_(0);
    ffy_.y = -1.0*f_(1);

      ffx.publish(ffx_);
      ffy.publish(ffy_);
     ui->qcustomplot3->graph(1)->setPen(pen);
     ui->qcustomplot4->graph(1)->setPen(pen);

}
void MainWindow::trigger_cb(const geometry_msgs::Point::ConstPtr& msg){

    QPen pen;
    pen.setColor(Qt::blue);
    pen.setWidth(2);
    ui->qcustomplot5->graph(0)->setPen(pen);
    pen.setColor(Qt::red);
    pen.setWidth(2);
    ui->qcustomplot5->graph(1)->setPen(pen);

    ui->qcustomplot5->graph(0)->addData(time, msg->y);
    ui->qcustomplot5->graph(1)->addData(time, msg->x);

    trig_.x=msg->x;
    trig_.y=msg->y;
    trig.publish(trig_);


}
void MainWindow::force1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    force = *msg;
    QPen pen;
    pen.setColor(Qt::blue);
    pen.setWidth(2);
    Eigen::Vector3d  data, adata;
    data<< msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z;
    adata = payload_link1_rotation * data;
    ui->qcustomplot->graph(0)->addData(time , -1.0*adata(0));
    ui->qcustomplot2->graph(0)->addData(time ,-1.0*adata(1));

    flx_.x = -1.0*adata(0);
    fly_.x = -1.0*adata(1);


     ui->qcustomplot->graph(0)->setPen(pen);
     ui->qcustomplot2->graph(0)->setPen(pen);

    //std::cout << "rtghrt"<<std::endl;
}
void MainWindow::desired_velocity_cb(const geometry_msgs::Point::ConstPtr &msg){
  //  geometry_msgs::Point desired_velocity = *msg;
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(5);
    ui->qcustomplot6->graph(0)->addData(time , msg->x);   // body linear x
    ui->qcustomplot7->graph(0)->addData(time ,msg->y);    // body angular y

    vc2_v_.y=msg->x;
    vc2_w_.y=msg->y;


    ui->qcustomplot6->graph(0)->setPen(pen);
    ui->qcustomplot7->graph(0)->setPen(pen);

}
void MainWindow::desired_force_cb(const geometry_msgs::Point::ConstPtr &msg){
    desired_force = *msg;
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(2);

    ui->qcustomplot->graph(1)->addData(time , msg->x);
    ui->qcustomplot2->graph(1)->addData(time ,msg->y);

    flx_.y=msg->x;
    fly_.y=msg->y;


    flx.publish(flx_);
    fly.publish(fly_);

    ui->qcustomplot->graph(1)->setPen(pen);
    ui->qcustomplot2->graph(1)->setPen(pen);

}
void MainWindow::plot_loop()
{

  if(ros::ok())
  {

  }
  else
  {
     QApplication::quit();
  }
}
geometry_msgs::Point  desired_point;
void MainWindow::desired_traj_cb(const  geometry_msgs::Point::ConstPtr& msg ){

    desired_point.x = -1.0*msg->x;

    desired_point.y = msg->y;

    desired_point.z = msg->z;

    ui->customplot->graph(1)->addData(time , msg->x);
    ui->customplot2->graph(1)->addData(time ,  msg->y);


    ui->customplot->graph(2)->addData(time , msg->x - point.x );
    ui->customplot2->graph(2)->addData(time ,  msg->y - point.y);

    px_.x = msg->x;
    px_.y = point.x;

    py_.x = msg->y;
    py_.y = point.y;

    px.publish(px_);
    py.publish(py_);

    conn1_curve_data.push_back(QCPCurveData(count ,msg->x, msg->y ));

    conn1_curve->data()->set(conn1_curve_data,true);


    //ui->customplot3->graph(1)->addData(time ,  msg->z);
   // path_curve->data()->add(QCPCurveData(count , ));

}
void MainWindow::link_cb(const gazebo_msgs::LinkStates::ConstPtr &msg){

    link_state = *msg;

    for(unsigned int i=0;i< link_state.name.size();i++ ){
        if(link_state.name[i].compare("payload::payload_rec_g_box")==0){

            Eigen::Vector3d c2_ , c2;
            c2<<c2_vel.x , c2_vel.y, 0;
            c2_ = rotation_matrix* c2;
            ui->qcustomplot8->graph(0)->addData(time,  c2_(0));
            ui->qcustomplot9->graph(0)->addData(time, -c2_(1));

            QPen ppen;
            ppen.setColor(Qt::blue);
            ppen.setWidth(4);
            ppen.setStyle(Qt::SolidLine);
             ui->qcustomplot8->graph(0)->setPen(ppen);
             ui->qcustomplot9->graph(0)->setPen(ppen);

             ppen.setColor(Qt::red);
             ppen.setWidth(4);
             ppen.setStyle(Qt::DotLine);
             ui->qcustomplot8->graph(1)->setPen(ppen);
             ui->qcustomplot9->graph(1)->setPen(ppen);


             Eigen::Vector3d linear,linear_;
             linear<<link_state.twist[i].linear.x,link_state.twist[i].linear.y,0;
                linear_  =  rotation_matrix*linear ;
             ui->qcustomplot8->graph(1)->addData(time, linear_(0));
             ui->qcustomplot9->graph(1)->addData(time, -linear_(1));

             vc2_vx_.x = c2_(0);  //estimate
             vc2_vx_.y = linear_(0); //groundtruth

             vc2_vy_.x = c2_(1);  //estimate
             vc2_vy_.y = linear_(1); //groundtruth

             vc2_vx.publish(vc2_vx_);
             vc2_vy.publish(vc2_vy_);





             Eigen::Vector3d vc2_, vc2_b;
             vc2_ << link_state.twist[i].linear.x, link_state.twist[i].linear.y, 0;

              ppen.setStyle(Qt::SolidLine);
              ppen.setColor(Qt::blue);
             vc2_b = rotation_matrix *vc2_;
             ui->qcustomplot10->graph(0)->setPen(ppen);
             ui->qcustomplot11->graph(0)->setPen(ppen);
             ui->qcustomplot10->graph(0)->addData(time, vc2_b(0));
             ui->qcustomplot11->graph(0)->addData(time, vc2_b(1));
             vc.x = vc2_b(0);
             vc.y = vc2_b(1);
             vcb.publish(vc);

        }
        if(link_state.name[i].compare("payload::payload_link2")==0){
             double x , y, z, w;
             x=link_state.pose[i].orientation.x;
             y=link_state.pose[i].orientation.y;
             z=link_state.pose[i].orientation.z;
             w=link_state.pose[i].orientation.w;
             payload_link1_rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                 2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                 2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


        }
        if(link_state.name[i].compare("payload::payload_link1")==0){
             double x , y, z, w;
             x=link_state.pose[i].orientation.x;
             y=link_state.pose[i].orientation.y;
             z=link_state.pose[i].orientation.z;
             w=link_state.pose[i].orientation.w;
             payload_link2_rotation<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
                 2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
                 2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;
        }

        //payload::connector
        if( link_state.name[i].compare("iris_1::base_link")==0){

           // conn1_curve_data.push_back(QCPCurveData(count ,link_state.pose[i].position.x,  link_state.pose[i].position.y ));
           // conn1_curve->data()->set(conn1_curve_data,true);
        }
        if( link_state.name[i].compare("payload::payload_rec_g_box")==0){

            conn2_curve_data.push_back(QCPCurveData(count ,link_state.pose[i].position.x,  link_state.pose[i].position.y ));
            conn2_curve->data()->set(conn2_curve_data,true);


            Eigen::Matrix3d payload_Rotation;


            double x , y, z, w;
            double roll,pitch,yaw;
            Eigen::Matrix3d RB;
            Eigen::Vector3d v,v_;
            x=link_state.pose[i].orientation.x;
            y=link_state.pose[i].orientation.y;
            z=link_state.pose[i].orientation.z;
            w=link_state.pose[i].orientation.w;
            tf::Quaternion Q(x,y,z,w   );

            tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);

            v_ <<  link_state.twist[i].linear.x,
                    link_state.twist[i].linear.y,
                    0;
            RB<< cos(yaw), sin(yaw) , 0,
                 -sin(yaw), cos(yaw), 0,
                    0,0,1;

               v =  RB * v_ ;
//            tf::Quarternion Q(x,y,z,w);
//            tf::Quart
            double vel = v(0);
            QPen ppen;
            ppen.setColor(Qt::blue);
            ppen.setWidth(5);
            ppen.setStyle(Qt::DotLine);
            ui->qcustomplot6->graph(1)->addData(time, c2_vel_b.x  );
            ui->qcustomplot7->graph(1)->addData(time, c2_vel_b.z );

            vc2_v_.x =  c2_vel_b.x ;
            vc2_w_.x =  c2_vel_b.z ;
            vc2_v.publish(vc2_v_);
            vc2_w.publish(vc2_w_);

             ui->qcustomplot6->graph(1)->setPen(ppen);
             ui->qcustomplot7->graph(1)->setPen(ppen);

//            point.x = link_state.pose[i].position.x;
//            point.y = link_state.pose[i].position.y;
//            point.z = link_state.pose[i].position.z;
//            ui->customplot->graph(0)->addData(time,point.x);
//            ui->customplot2->graph(0)->addData(time,point.y);

             desired_curve_data.push_back(QCPCurveData(count ,link_state.pose[i].position.x,  link_state.pose[i].position.y ));
             desired_curve->data()->set(desired_curve_data,true);




             map_desired_curve_data.push_back(QCPCurveData(count ,link_state.pose[i].position.x,  link_state.pose[i].position.y ));
             map_desired_curve->data()->set(map_desired_curve_data,true);




             payload_vel.x  = link_state.twist[i].linear.x;
             payload_vel.y  = link_state.twist[i].linear.y;
             payload_vel.z  = link_state.twist[i].linear.z;
             payload_pose.x = link_state.pose[i].position.x;
             payload_pose.y = link_state.pose[i].position.y;
             payload_pose.z = link_state.pose[i].position.z;
             point.x = payload_pose.x;
             point.y = payload_pose.y;
             point.z = payload_pose.z;
             ui->customplot->graph(0)->addData(time,point.x);
             ui->customplot2->graph(0)->addData(time,point.y);




        }
    }



}



void MainWindow::model_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    double dt = ros::Time::now().toSec() - last_ros_time;
    if(dt >100){
      dt=0;
    }
    last_ros_time = ros::Time::now().toSec();
    time+=dt;
    model_state = *msg;

    for(unsigned int i=0;i<model_state.name.size();i++ ){
        if(model_state.name[i].compare("payload")==0){


            //rotation_matrix

            tf::Quaternion q(
                        model_state.pose[i].orientation.x,
                        model_state.pose[i].orientation.y,
                        model_state.pose[i].orientation.z,
                        model_state.pose[i].orientation.w
                        );
            double r,p,y;
            tf::Matrix3x3(q).getRPY(r,p,y);
            rotation_matrix<< cos(y), sin(y),0,
                              -sin(y), cos(y),0,
                                0,         0,   1;

            //            desired_curve_data.push_back(QCPCurveData(count ,model_state.pose[i].position.x,  model_state.pose[i].position.y ));
            //            desired_curve->data()->set(desired_curve_data,true);

            //            payload_vel.x  = model_state.twist[i].linear.x;
            //            payload_vel.y  = model_state.twist[i].linear.y;
            //            payload_vel.z  = model_state.twist[i].linear.z;
//                        payload_pose.x = model_state.pose[i].position.x;
//                        payload_pose.y = model_state.pose[i].position.y;
//                        payload_pose.z = model_state.pose[i].position.z;
            //            point.x = payload_pose.x;
            //            point.y = payload_pose.y;
            //            point.z = payload_pose.z;
            //            ui->customplot->graph(0)->addData(time,point.x);
            //            ui->customplot2->graph(0)->addData(time,point.y);


        }
    }


    if((tick %100)==0){
        QCPItemLine   *line;
        line = new  QCPItemLine(ui->customplot4);
        double strength = sqrt(payload_vel.x * payload_vel.x + payload_vel.y * payload_vel.y);
        double unit_x =  (payload_vel.x)  /strength;
        double unit_y =  (payload_vel.y)  /strength;
        line->start->setCoords(payload_pose.x , payload_pose.y);

        line->end->setCoords( payload_vel.x+ payload_pose.x , payload_vel.y+payload_pose.y);
        line->setHead(QCPLineEnding::esSpikeArrow);

        QPen arrow_pen;
        //line.setWidth(1);
        arrow_pen.setWidth(2);
        arrow_pen.setColor(Qt::red);
        line->setPen(arrow_pen);
        line->setVisible(true);
        arrow.push_back(line);
    }

    tick++;

    path_curve_data.push_back(QCPCurveData(count , - desired_point.x ,   desired_point.y));
    path_curve->data()->set(path_curve_data,true);
    map_path_curve_data.push_back(QCPCurveData(count, -desired_point.x, desired_point.y));
    map_path_curve->data()->set(map_path_curve_data);

    count++;

    ui->customplot->replot();
    ui->customplot2->replot();
    ui->customplot3->replot();
    ui->customplot4->replot();
    ui->qcustomplot->replot();
    ui->qcustomplot2->replot();

    ui->qcustomplot3->replot();
    ui->qcustomplot4->replot();
    ui->qcustomplot5->replot();
    ui->qcustomplot6->replot();
    ui->qcustomplot7->replot();
    ui->qcustomplot8->replot();
    ui->qcustomplot9->replot();
    ui->plot->replot();
    ui->qcustomplot10->replot();
    ui->qcustomplot11->replot();
    ui->customplot->replot();
    ui->customplot2->replot();
}

void MainWindow::point_cb(const geometry_msgs::Point::ConstPtr & msg){



}


bool MainWindow::getRosTopics(ros::master::V_TopicInfo& topics){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    std::string str;
    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed!" << std::endl;
        return false;
    }

    topics.clear();
    for (int i = 0; i < payload.size(); ++i)
        topics.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
    return true;
}


void MainWindow::on_checkBox_stateChanged(int arg1)
{
  std::cout<<ui->checkBox->isChecked()<<std::endl;
  if(ui->checkBox->isChecked()){
    autoscroll = true;
  }else{
    autoscroll = false;
  }

}

void MainWindow::on_pushButton_clicked()
{

  QString filename = QFileDialog::getSaveFileName(0,"Save file",QDir::currentPath(),
         "png files (*.png)",
             new QString("png files (*.png)"));
  ui->customplot->savePng(filename+"0.png", 0, 0, 5,100, -1);
  ui->customplot2->savePng(filename+"1.png", 0, 0, 5,100, -1);
  ui->customplot3->savePng(filename+"2.png", 0, 0, 2,100, -1);
  ui->customplot4->savePng(filename+"4.png", 0, 0, 2,100, -1);

  ui->qcustomplot->savePng(filename+"5.png", 0, 0, 2,100, -1);
  ui->qcustomplot2->savePng(filename+"6.png", 0, 0, 2,100, -1);

  ui->qcustomplot3->savePng(filename+"7.png", 0, 0, 2,100, -1);
  ui->qcustomplot4->savePng(filename+"8.png", 0, 0, 2,100, -1);
  ui->qcustomplot5->savePng(filename+"9.png", 0, 0, 2,100, -1);
  ui->qcustomplot6->savePng(filename+"10.png", 0, 0, 2,100, -1);
  ui->qcustomplot7->savePng(filename+"11.png", 0, 0, 2,100, -1);
  ui->qcustomplot8->savePng(filename+"12.png", 0, 0, 2,100, -1);
  ui->qcustomplot9->savePng(filename+"13.png", 0, 0, 2,100, -1);
  ui->qcustomplot10->savePng(filename+"14.png", 0, 0, 2,100, -1);
  ui->qcustomplot11->savePng(filename+"15.png", 0, 0, 2,100, -1);
  ui->plot->savePng(filename+"16.png", 0, 0, 2,100, -1);
}

void MainWindow::on_pushButton_2_clicked()
{
  if(!record){
    record = true;
    state_timer->start();

  }else{
    record = false;
    state_timer->stop();

  }
}

void MainWindow::on_maxspinbox_valueChanged(const QString &arg1)
{
    double max = arg1.toDouble();
    std::cout <<"max" <<max<<std::endl;
    t_max = max;
    ui->customplot2->xAxis->setRange(t_min,t_max);
    ui->customplot->xAxis->setRange(t_min,t_max);
    ui->customplot3->xAxis->setRange(t_min,t_max);
    ui->customplot->replot();
    ui->customplot2->replot();
    ui->customplot3->replot();
}

void MainWindow::on_minspinbox_valueChanged(const QString &arg1)
{
  double min = arg1.toDouble();
  std::cout <<"min" <<min<<std::endl;
  t_min = min;
  ui->customplot2->xAxis->setRange(t_min,t_max);
  ui->customplot->xAxis->setRange(t_min,t_max);
   ui->customplot3->xAxis->setRange(t_min,t_max);

  ui->customplot->replot();
   ui->customplot2->replot();
    ui->customplot3->replot();
}
