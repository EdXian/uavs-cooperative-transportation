#include <ros/ros.h>

#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/ModelState.hh>
#include <gazebo/physics/LinkState.hh>




geometry_msgs::Point break_joint, uav2_psi_groundtruth;
void break_joint_cb(const geometry_msgs::Point::ConstPtr& msg){
  break_joint = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tttt");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}




namespace gazebo
{

class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    this->world = _world;
    this->updateRate = common::Time(0,common::Time::SecToNano(3));
    this->prevUpdateTime = common::Time::GetWallTime();
    this->rosSub = this->rosNode.subscribe<geometry_msgs::Point>("/break_joint", 2, break_joint_cb);
    this->rosPub = this->rosNode.advertise<geometry_msgs::Point>("/uav2_psi_groundtruth", 2);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
               boost::bind(&WorldPluginTutorial::OnUpdate, this, _1));
    //this->iris_model = this->world->GetModel("iris1");
    //this->iris_base_link = this->iris_model->GetLink("iris1::base_link");
  }
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {

    if(common::Time::GetWallTime() - this->prevUpdateTime > 15){
      this->payload_model =  this->world->ModelByName("payload");
      this->payload_g = this->payload_model->GetLink("payload::payload_rec_g_box");
      this->payload_g1 = this->payload_model->GetLink("payload::payload_rec_g1_box");

}

  //Double drones add joint

double x;


    if(common::Time::GetWallTime() - this->prevUpdateTime > 10 && add_inv_2){
    this->iris_model =  this->world->ModelByName("firefly2");
    this->iris_base_link = this->iris_model->GetLink("firefly2::connector1");
    this->payload_model =  this->world->ModelByName("payload");


    this->payload_link = this->payload_model->GetLink("payload::payload_link1_box");
    this->joint_ = this->world.get()->Physics()->CreateJoint("ball", this->iris_model);
//    this->joint_ = this->world->GetPhysicsEngine()->CreateJoint("revolute2", this->iris_model);


    this->joint_->Load(this->iris_base_link,this->payload_link,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_->Attach(this->iris_base_link,this->payload_link);
    ignition::math::Vector3d joint_axis(0,1,0), joint_axis2(1,0,0),joint_axis3(0,0,1);
    this->joint_->SetAxis(0, joint_axis2);
    this->joint_->SetAxis(1, joint_axis);
    this->joint_->SetAxis(2, joint_axis3);
    this->joint_->SetName("payload_drone_joint");
    ROS_INFO("add joint1");

    this->iris_model2=  this->world->ModelByName("firefly1");
    this->iris_base_link2 = this->iris_model2->GetLink("firefly1::connector1");

    this->payload_model2  =  this->world->ModelByName("payload");
    this->payload_model2->SetStatic(false);

    this->payload_link2 = this->payload_model2->GetLink("payload::payload_link2_box");
    this->joint_2 = this->world.get()->Physics()->CreateJoint("ball", this->iris_model2);

    this->joint_2->Load(this->payload_link2, this->iris_base_link2,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_2->Attach(this->payload_link2,this->iris_base_link2);
    this->joint_2->SetAxis(0, joint_axis2);
    this->joint_2->SetAxis(1, joint_axis);
    this->joint_2->SetAxis(2, joint_axis3);

    this->joint_2->SetName("payload_drone_joint2");
    ROS_INFO("add joint2");
     add_inv_2 = false;
ros::spinOnce();

}

  }
    private: physics::WorldPtr world;

    private: event::ConnectionPtr updateConnection;
    common::Time updateRate;
    common::Time prevUpdateTime;
    //For double drones
    private: physics::JointPtr joint_, joint_2;
    private: physics::ModelPtr iris_model, iris_model2;
    private: physics::ModelPtr payload_model, payload_model2;
    private: physics::LinkPtr iris_base_link, iris_base_link2;
    private: physics::LinkPtr payload_link, payload_link2, payload, payload_box2, payload_g, payload_g1;
    bool add_inv = true;
    bool add_inv_2 = true;
    //For single drones
    private: physics::JointPtr joint_3, joint_4, joint_5;
    private: physics::ModelPtr iris_model3;
    private: ros::NodeHandle rosNode;
    private: ros::Subscriber rosSub;
    private: ros::Publisher rosPub;


};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
