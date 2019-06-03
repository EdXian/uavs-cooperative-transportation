//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

//#include "geometric_controller/geometric_controller.h"
#include "geometric_controller.h"
using namespace Eigen;
using namespace std;
//Constructor

geometricCtrl::geometricCtrl():

  fail_detec_(false),
  ctrl_enable_(true),
  landing_commanded_(false),
  feedthrough_enable_(false),
  node_state(WAITING_FOR_HOME_POSE),

  mass(deafault_mass)
{


    mav_name_= "iris1";
    ctrl_mode_= MODE_BODYRATE;
    sim_enable_= true;
    max_fb_acc_= 7.0;
    mavYaw_= 0.0;
    dx_= 0.0;
    dy_= 0.0;
    dz_= 0.0;
    attctrl_tau_= 0.12;  //tau=0.2
    norm_thrust_const_= 0.04; // 1 / max acceleration

    Kpos_x_= 6.0;
    Kpos_y_= 6.0;
    Kpos_z_= 10.0;
    Kvel_x_= 1.5;
    Kvel_y_= 1.5;
    Kvel_z_= 3.33;

    use_dob_= false;
    a0_x= 10.0;
    a0_y= 10.0;
    a0_z= 10.0;
    a1_x= 10.0;
    a1_y= 10.0;
    a1_z= 10.0;
    a1_y= 10.0;
    a1_z= 10.0;
    tau_x= 10.0;
    tau_y= 10.0;
    tau_z=10.0;
    dhat_max= 10.0;
    dhat_min= -10.0;


    desired_force <<0,0,0;
}
geometricCtrl::~geometricCtrl() {
  //Destructor
}

void geometricCtrl::set_cmd(Eigen::Vector3d pos,Eigen::Vector3d vel, Eigen::Vector3d acc ){
    targetPos_ =pos; //Initial Position
    targetVel_ =vel;
    targetAcc_ =acc;
}
void geometricCtrl::set_init(){

    targetPos_ << 0.0, 0.0, 2.0; //Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    targetAcc_ << 0, 0, 0;

    g_ << 0.0, 0.0, -9.8;

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
    D_ << dx_, dy_, dz_;

    q_.resize(3);
    p_.resize(3);
    for (unsigned int i = 0; i < 3; ++i) {
      q_.at(i) << 0.0, 0.0;
      p_.at(i) << 0.0, 0.0;
    }
    a0 << a0_x, a0_y, a0_z;
    a1 << a1_x, a1_y, a1_z;
    tau << tau_x, tau_y, tau_z;

}

void geometricCtrl::set_pose(Eigen::Vector3d pose, Eigen::Vector4d ori, Eigen::Vector3d vel, Eigen::Vector3d ang){
    mavAtt_  = ori;
    mavPos_  = pose;
    mavVel_  = vel;
    mavRate_ = ang;
}

void geometricCtrl::computeBodyRateCmd(int mode){
  Eigen::Vector3d errorPos_, errorVel_;
  Eigen::Matrix3d R_ref;

  errorPos_ = mavPos_ - targetPos_;
  errorVel_ = mavVel_ - targetVel_;
  a_ref = targetAcc_;

  if(mode  == 1 ){       //use_dob_
    /// Compute BodyRate commands using disturbance observer
    /// From Hyuntae Kim
    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    ///
    q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_; //feedforward term for trajectory error

    //a_dob = disturbanceobserver(errorPos_, a_ref + a_fb - a_dob);
    //a_des = a_ref + a_fb - a_dob - g_;

    //a_des = a_ref + a_fb  - g_;

    a_des = mass* a_ref + a_fb  - mass * g_;

    q_des = acc2quaternion(a_des, mavYaw_);

  } else if(mode==3){
    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    R_ref = quat2RotMatrix(q_ref);
    a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_; //feedforward term for trajectory error
    if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; //Clip acceleration if reference is too large
    a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; //Rotor drag
    a_des = a_fb + a_ref - a_rd - g_;
    q_des = acc2quaternion(a_des, mavYaw_);

  }else if(mode ==2){
      q_ref = acc2quaternion(a_ref - g_, mavYaw_);
      a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_; //feedforward term for trajectory error

      //a_dob = disturbanceobserver(errorPos_, a_ref + a_fb - a_dob);
      //a_des = a_ref + a_fb - a_dob - g_;

      //a_des = a_ref + a_fb  - g_;

      a_des = desired_force - mass * g_;

      q_des = acc2quaternion(a_des, mavYaw_);

  }

  cmdBodyRate_ = attcontroller(q_des, a_des, mavAtt_); //Calculate BodyRate
}

Eigen::Vector4d geometricCtrl::quatMultiplication(Eigen::Vector4d &q, Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d geometricCtrl::quat2RotMatrix(Eigen::Vector4d q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d geometricCtrl::rot2Quaternion(Eigen::Matrix3d R){
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(Eigen::Vector3d vector_acc, double yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, yc;
  Eigen::Matrix3d rotmat;
  yc = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitY();
  zb_des = vector_acc / vector_acc.norm();
  xb_des = yc.cross(zb_des) / ( yc.cross(zb_des) ).norm();
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}


void  geometricCtrl::set_payload_info(Eigen::Vector3d T ){

desired_force = T;


}




Eigen::Vector4d geometricCtrl::attcontroller(Eigen::Vector4d &ref_att, Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att){
  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, q_bar, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;
  inverse << 1.0, -1.0, -1.0, -1.0;


  q_inv = inverse.asDiagonal() * curr_att;
  //  q_bar = q_inv/q_inv.norm();
  //  we can ignore ||q|| because the value of ||q|| is 1.
  qe = quatMultiplication(  q_inv, ref_att);

  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);

  rotmat = quat2RotMatrix(mavAtt_);

  zb = rotmat.col(2); //The unit-z vector represnts in inertial frame.
  zb = rotmat.col(2);

  //ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); //Calculate thrust

  //The desired thrust should be in the interval [0,1].
  //ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); //Calculate thrust


  //solve a quadratic equation  20T^2+15T^2-f = 0 where T is the thrust of drone.
  //The coefficient of the quadratic equation is determined by RLS method.


  double thrust = ref_acc.dot(zb);

  double a=29,b=10,c = -1.0000000*thrust;
  double x =0;

  std::vector<double> root;
  if((b*b - 4*a*c)>0){

      double sqrt_ =  sqrt(b*b - 4.00000*a*c);
      root.push_back(  (    -1.0000000 * b+     sqrt_) /(2.0000000*a));
      root.push_back(  (    -1.0000000 * b-     sqrt_) /(2.0000000*a));
  }

    for(unsigned int i=0;i<root.size();i++){
        if(root[i]>0){
          ratecmd(3) = root[i];
        }
    }

  return ratecmd;

}












///
void geometricCtrl::getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel){
  pos = mavPos_;
  att = mavAtt_;
  vel = mavVel_;
  angvel = mavRate_;
}

Eigen::Vector3d geometricCtrl::disturbanceobserver(Eigen::Vector3d pos_error, Eigen::Vector3d acc_setpoint){

  Eigen::Vector3d acc_input, yq, yp, d_hat;
  double control_dt = 0.01;

  for(int i = 0; i < acc_input.size(); i++){
    //Update dob states
    p_.at(i)(0) = p_.at(i)(0) + p_.at(i)(1) * control_dt;
    p_.at(i)(1) = (-a0(i) * control_dt / std::pow(tau(i),2)) * p_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) *p_.at(i)(1) + control_dt * acc_setpoint(i);
    q_.at(i)(0) = q_.at(i)(0) + control_dt * q_.at(i)(1);
    q_.at(i)(1) = (-a0(i)/std::pow(tau(i), 2)) * control_dt * q_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) * q_.at(i)(1) + control_dt * pos_error(i);

    //Calculate outputs
    yp(i) = (a0(i) / pow(tau(i), 2)) * p_.at(i)(0);
    yq(i) = (-a1(i)*a0(i) / std::pow(tau(i), 3))*q_.at(i)(0) - (std::pow(a0(i),2) / std::pow(tau(i), 4)) * q_.at(i)(1) + a0(i) / pow(tau(i),2) * pos_error(i);
    d_hat(i) = yq(i) - yp(i);
    d_hat(i) = std::max( std::min( d_hat(i), dhat_max), dhat_min);
    acc_input(i) = d_hat(i);
  }

  return acc_input;
}

void geometricCtrl::setBodyRateCommand(Eigen::Vector4d bodyrate_command){
  cmdBodyRate_= bodyrate_command;

}

void geometricCtrl::setFeedthrough(bool feed_through){
  feedthrough_enable_ = feed_through;

}
