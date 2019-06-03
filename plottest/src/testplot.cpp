#include"stdio.h"
#include"ros/ros.h"
#include"GUI/mainwindow.h"
#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>
#include "GUI/qcustomplot.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "testplot");
  QApplication app(argc, argv);
  MainWindow window;
  window.show();
 return app.exec();
}
