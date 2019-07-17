#include "path.h"
#include <tf2/transform_datatypes.h>
using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
    Node3D node;
    path.poses.clear();
    pathNodes.markers.clear();
    pathVehicles.markers.clear();
    pathVehicletexts.markers.clear();
    pathpayloads.markers.clear();
    pathleaderuav.markers.clear();
    pathfolloweruav.markers.clear();
    pathleaderuav_text.markers.clear();
    pathfolloweruav_text.markers.clear();
    pathdots.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Node3D> nodePath) {
  path.header.stamp = ros::Time::now();

  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i]);
    plotdot(nodePath[i].getX() * Constants::cellSize ,nodePath[i].getY() * Constants::cellSize , i);

    k++;
    if((i%15== 0)|| i== (nodePath.size()-1)){
        addVehicle(nodePath[i], k);
        addNode(nodePath[i], k);
    }
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;

  path.poses.push_back(vertex);


}

// ________
// ADD NODE
void Path::addNode(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;
  double w_,x_,y_,z_ , r,p,yaw;
  x_=orientaiotn.x;
  y_=orientaiotn.y;
  z_=orientaiotn.z;
  w_=orientaiotn.w;
  tf::Quaternion Q(
  x_,
  y_,
  z_,
  w_   );
  tf::Matrix3x3(Q).getRPY(r,p,yaw);
  //geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(r,p,-yaw);
  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::ARROW;
  pathNode.scale.x = 0.05;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.13;
  pathNode.color.a = 0.5;
  double offx, offy;
  offx = - cos(yaw) *2.0/2;
  offy = - sin(yaw) *2.0/2;
    pathNode.points.clear();
    start_point.x+=0.4*offx;
    start_point.y+=0.4*offy;
    start_point.z = 0.4;
    end_point.x+=1.2*offx;
    end_point.y+=1.2*offy;
    end_point.z=0.4;
  pathNode.points.push_back(end_point);
  pathNode.points.push_back(start_point);

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

//  pathNode.pose.position.x = node.getX() * Constants::cellSize- cos(yaw) *2.0/2;
//  pathNode.pose.position.y = node.getY() * Constants::cellSize- sin(yaw) *2.0/2;
  //pathNode.pose.orientation =orientaiotn;



  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;
  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 0.2;
  pathVehicle.color.a = 0.1;


  //  pathVehicle .
  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  orientaiotn =  tf::createQuaternionMsgFromYaw(node.getT());

  //pathVehicles.markers.push_back(pathVehicle);

  vehicle_text(pathVehicle.pose.position.x ,pathVehicle.pose.position.y , i);

    payload(pathVehicle.pose.position.x ,pathVehicle.pose.position.y , i);
    leader_uav(pathVehicle.pose.position.x ,pathVehicle.pose.position.y , i);
    follower_uav(pathVehicle.pose.position.x ,pathVehicle.pose.position.y , i);
}


void Path::leader_uav(double x, double y, int i){

     visualization_msgs::Marker uav;
     visualization_msgs::Marker uav_text;

     double w_,x_,y_,z_ , r,p,yaw;
     x_=orientaiotn.x;
     y_=orientaiotn.y;
     z_=orientaiotn.z;
     w_=orientaiotn.w;
     tf::Quaternion Q(
     x_,
     y_,
     z_,
     w_   );
     tf::Matrix3x3(Q).getRPY(r,p,yaw);


     uav.ns = "basic_shapes";

     uav.header.frame_id = "path";
     uav.header.stamp = ros::Time::now();
     uav.id = i;
     uav.type = visualization_msgs::Marker::MESH_RESOURCE;
     //uav.action = visualization_msgs::Marker::ADD;
     uav.mesh_resource = "package://rotors_description/meshes/firefly.dae";
     //uav.type = visualization_msgs::Marker::SPHERE;
     uav.scale.x = 1.0;
     uav.scale.y = 1.0;
     uav.scale.z = 1.0;
     uav.color.a = 1.0;
     uav.color.b = 1.0;
     uav.color.g = 0.0;
     uav.color.r = 0.0;

     uav.lifetime = ros::Duration();
     uav.pose.position.x = x + cos(yaw) *1.2/2   ;
     uav.pose.position.y = y + sin(yaw)*1.2/2;
     uav.pose.position.z = 0.4;

     end_point.x =  uav.pose.position.x;
     end_point.y =  uav.pose.position.y;

     uav.pose.orientation = orientaiotn;
     pathleaderuav.markers.push_back(uav);

     uav_text.ns = "basic_shapes";
     uav_text.header.frame_id = "path";
     uav_text.header.stamp = ros::Time::now();
     uav_text.id = i;
     uav_text.action = visualization_msgs::Marker::ADD;
     uav_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

     uav_text.color.r = 255;
     uav_text.color.g = 0;
     uav_text.color.b = 0;
     uav_text.scale.x = 0.1;
     uav_text.scale.y = 0.1;
     uav_text.scale.z = 0.1;
     uav_text.color.a = 1;
     uav_text.lifetime = ros::Duration();
     uav_text.text = "Follower";
     uav_text.pose.position.x = x + cos(yaw) *1.5/2   ;
     uav_text.pose.position.y = y + sin(yaw)*1.5/2;

     pathleaderuav_text.markers.push_back(uav_text);


}

void Path::follower_uav(double x, double y, int i){
    visualization_msgs::Marker uav;
    visualization_msgs::Marker uav_text;

    double w_,x_,y_,z_ , r,p,yaw;
    x_=orientaiotn.x;
    y_=orientaiotn.y;
    z_=orientaiotn.z;
    w_=orientaiotn.w;
    tf::Quaternion Q(
    x_,
    y_,
    z_,
    w_   );
    tf::Matrix3x3(Q).getRPY(r,p,yaw);
    uav.ns = "basic_shapes";
    uav.header.frame_id = "path";
    uav.header.stamp = ros::Time::now();
    uav.id = i;
    uav.type = visualization_msgs::Marker::MESH_RESOURCE;
    uav.mesh_resource = "package://rotors_description/meshes/firefly.dae";
    uav.scale.x = 1.0;
    uav.scale.y = 1.0;
    uav.scale.z = 1.0;
    uav.color.a = 1.0;
    uav.lifetime = ros::Duration();
    uav.pose.position.x = x - cos(yaw) *1.2/2   ;
    uav.pose.position.y = y - sin(yaw)*1.2/2;
    uav.pose.position.z = 0.4;
    start_point.x =  uav.pose.position.x;
    start_point.y =  uav.pose.position.y;
    uav.pose.orientation = orientaiotn;
    pathfolloweruav.markers.push_back(uav);

    uav_text.ns = "basic_shapes";
    uav_text.header.frame_id = "path";
    uav_text.header.stamp = ros::Time::now();
    uav_text.id = i;
    uav_text.action = visualization_msgs::Marker::ADD;
    uav_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    uav_text.color.r = 255;
    uav_text.color.g = 0;
    uav_text.color.b = 0;
    uav_text.scale.x = 0.1;
    uav_text.scale.y = 0.1;
    uav_text.scale.z = 0.1;
    uav_text.color.a = 1;
    uav_text.lifetime = ros::Duration();
    uav_text.text = "Leader";
    uav_text.pose.position.x = x - cos(yaw) *1.5/2   ;
    uav_text.pose.position.y = y - sin(yaw)*1.5/2;

    pathfolloweruav_text.markers.push_back(uav_text);



}
void Path::plotdot(double x, double y, int i){
    visualization_msgs::Marker dot;
    dot.header.frame_id = "path";
    dot.header.stamp = ros::Time(0);
    dot.id = i;
    dot.type = visualization_msgs::Marker::SPHERE;
    dot.scale.x = 0.1;
    dot.scale.y = 0.1;
    dot.scale.z = 0.1;
    dot.color.a = 1;


    dot.color.r = Constants::teal.red;
    dot.color.g = Constants::teal.green;
    dot.color.b = Constants::teal.blue;

    dot.pose.position.x = x;
    dot.pose.position.y = y;
    dot.pose.position.z = 0.0;
    dot.pose.orientation = orientaiotn;
    pathdots.markers.push_back(dot);

}

void Path::payload(double x, double y , int i){
    visualization_msgs::Marker payload_marker;

    payload_marker.header.frame_id = "path";
    payload_marker.header.stamp = ros::Time(0);
    payload_marker.id = i;
    payload_marker.type = visualization_msgs::Marker::CUBE;
    payload_marker.scale.x = Constants::length - 0.15;
    payload_marker.scale.y = Constants::width - 0.4;
    payload_marker.scale.z = 0.1;
    payload_marker.color.a = 1.0;


    payload_marker.color.r = Constants::teal.red;
    payload_marker.color.g = Constants::teal.green;
    payload_marker.color.b = Constants::teal.blue;

    payload_marker.pose.position.x = x;
    payload_marker.pose.position.y = y;
    payload_marker.pose.position.z = 0.3;
    payload_marker.pose.orientation = orientaiotn;
    pathpayloads.markers.push_back(payload_marker);


}








void Path::vehicle_text(double x, double y,int i){
    visualization_msgs::Marker pathvehicletext;
    pathvehicletext.ns = "basic_shapes";
    pathvehicletext.header.frame_id = "path";
    pathvehicletext.header.stamp = ros::Time::now();
    pathvehicletext.id = i;
    pathvehicletext.action = visualization_msgs::Marker::ADD;
    pathvehicletext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    pathvehicletext.color.r = 255;
    pathvehicletext.color.g = 0;
    pathvehicletext.color.b = 0;
    pathvehicletext.scale.x = 0.1;
    pathvehicletext.scale.y = 0.1;
    pathvehicletext.scale.z = 0.1;
    pathvehicletext.color.a = 1;
    pathvehicletext.lifetime = ros::Duration();
    pathvehicletext.text = "Payload";
    pathvehicletext.pose.position.x = x;
    pathvehicletext.pose.position.y = y;

    pathVehicletexts.markers.push_back(pathvehicletext);
}
