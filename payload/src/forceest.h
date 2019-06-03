#ifndef FORCEEST_H
#define FORCEEST_H
#include "ukf.h"
#include "iostream"
#include "lpf2.h"



// add wx wy wz
enum state{
    rp_x=0,
    rp_y,
    rp_z,

    omega_r,
    omega_p,
    omega_y,

    alpha_r,
    alpha_p,
    alpha_y,

    ac_x,
    ac_y,
    ac_z,

    ap_x,
    ap_y,
    ap_z,

    FF_x,
    FF_y,
    FF_z,

    FL_x,
    FL_y,
    FL_z,
    statesize
};

enum measurement{

    mrpx=0,
    mrpy,
    mrpz,

    momegar,
    momegap,
    momegay,

    malphar,
    malphap,
    malphay,

    mac_x,
    mac_y,
    mac_z,

    mFF_x,
    mFF_y,
    mFF_z,
    measurementsize
};


class forceest : public ukf
{

public:
  lpf2 *lpf;
forceest(int x, int y) : ukf(x,y){
 last_omega_p = 0;
 lpf = new lpf2(6,0.02);
}

Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);
double last_omega_p ;
private:


};

#endif // FORCEEST_H
