#include "bagconv.h"

bagconv::bagconv()
{

}
void bagconv::force1_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  Eigen::Vector3d  data, adata;
  data<< msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z;
  adata = payload_link2_rotation * data;

  //publish -adata(0) -adata(1)


}
void bagconv::force2_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  Eigen::Vector3d data , adata;
  data <<msg->wrench.force.x , msg->wrench.force.y ,msg->wrench.force.z;
  adata = payload_link1_rotation*data;
  data = rotation_matrix * adata;
   //publish adata(0) adata(1)
}

void bagconv::model_cb(const gazebo_msgs::ModelStates::ConstPtr &msg){
  for(unsigned int i=0;i<model_state.name.size();i++ ){
      if(model_state.name[i].compare("payload")==0){
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
      }
  }
}



void bagconv::vel_est_b_cb(const geometry_msgs::Point::ConstPtr &msg){
    c2_vel_b = *msg;


}


void bagconv::vel_est_cb(const  geometry_msgs::Point::ConstPtr& msg){
    c2_vel = *msg;


}



void bagconv::link_cb(const gazebo_msgs::LinkStates::ConstPtr &msg){
  link_state = *msg;

  for(unsigned int i=0;i< link_state.name.size();i++ ){
      if(link_state.name[i].compare("payload::payload_rec_g_box")==0){

          Eigen::Vector3d c2_ , c2;
          c2<<c2_vel.x , c2_vel.y, 0;
          c2_ = rotation_matrix* c2;
          //publish c2_(0) c2_(1)  (estimate)

           Eigen::Vector3d linear,linear_;
          linear<<link_state.twist[i].linear.x,link_state.twist[i].linear.y,0;
          linear_  =  rotation_matrix*linear ;

           Eigen::Vector3d vc2_, vc2_b;
           vc2_ << link_state.twist[i].linear.x, link_state.twist[i].linear.y, 0;

           vc2_b = rotation_matrix *vc2_;
           //publish vc2_b(0) vc2_b(1) (groundtruth)
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


      if( link_state.name[i].compare("payload::payload_rec_g_box")==0){
          //publish this point (c2 position)


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
          double vel = v(0);

          //publish c2_vel_b.x c2_vel_b.z
//          ui->qcustomplot6->graph(1)->addData(time, c2_vel_b.x  );
//          ui->qcustomplot7->graph(1)->addData(time, c2_vel_b.z );


           payload_vel.x  = link_state.twist[i].linear.x;
           payload_vel.y  = link_state.twist[i].linear.y;
           payload_vel.z  = link_state.twist[i].linear.z;
           payload_pose.x = link_state.pose[i].position.x;
           payload_pose.y = link_state.pose[i].position.y;
           payload_pose.z = link_state.pose[i].position.z;


           point.x = payload_pose.x;
           point.y = payload_pose.y;
           point.z = payload_pose.z;

           //publish point.x point.y  (c2_ground_truth)

      }
  }
}


int main(int argc, char **argv){

  ros::init(argc, argv, "rosbagconv");




}
