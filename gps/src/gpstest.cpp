#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/UTMUPS.hpp>





using namespace  GeographicLib;

geometry_msgs::Point global_position;

//void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){

//    double northing , easting ;
//    char zone;



//    try {
//        {
//          double lat = 33.3, lon = 44.4; // Baghdad
//          int zone;
//          bool northp;
//          double x, y;
//          UTMUPS::Forward(lat, lon, zone, northp, x, y);
//          std::string zonestr = UTMUPS::EncodeZone(zone, northp,true);
//         //std::cout << fixed << setprecision(2) << zonestr << " " << x << " " << y << "\n";

//        }
//        {
//          // Sample reverse calculation
//          std::string zonestr = "38n";
//          int zone;
//          bool northp;
//          UTMUPS::DecodeZone(zonestr, zone, northp);
//          double x = 444e3, y = 3688e3;
//          double lat, lon;
//          UTMUPS::Reverse(zone, northp, x, y, lat, lon);

//        }
//      }
//      catch (const std::exception& e) {
//        std::cerr << "Caught exception: " << e.what() << "\n";

//      }
//}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpstest");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  while(ros::ok()){


  //  const GeographicLib::UTMUPS& ups = GeographicLib::UTMUPS


    ros::spinOnce();
    loop_rate.sleep();
  }




  ROS_INFO("Hello world!");
}
