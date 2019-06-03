#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("~/mavros/vision_pose/pose", 2);
    sub_ = n_.subscribe("/vrpn_client_node/RigidBody1/pose", 2, &SubscribeAndPublish::callback, this);
  }

  void callback(const geometry_msgs::PoseStamped::ConstPtr& input)
  {
    pub_.publish(input);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
