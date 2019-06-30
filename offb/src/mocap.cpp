#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>



class mocapsubpub{

public:
    //ros::NodeHandle nh;
    mocapsubpub(){
        mocap_sub= nh.subscribe("/vrpn_client_ros",1,&mocapsubpub::mocap_cb, this);
        pose_pub= nh.advertise<geometry_msgs::PoseStamped>("/drone1/vision/pose",1);
    }

    void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        geometry_msgs::PoseStamped pose;
        pose =*msg;

        pose_pub.publish(pose);

    }


private:

    ros::NodeHandle nh;

    ros::Subscriber  mocap_sub;
    ros::Publisher  pose_pub;


};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap");

  mocapsubpub mocap_run;

    ros::spin();
}
