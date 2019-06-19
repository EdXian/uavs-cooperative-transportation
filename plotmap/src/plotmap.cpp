#include"stdio.h"
#include"ros/ros.h"
#include"mainwindow.h"
#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotmap");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}

