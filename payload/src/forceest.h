#ifndef FORCEEST_H
#define FORCEEST_H
#include "ukf.h"
#include "iostream"
#include "lpf2.h"



// add wx wy wz
enum state{
    pc1_x=0,
    pc1_y,
    pc1_z,

    vc1_x,
    vc1_y,
    vc1_z,

    ac1_x,
    ac1_y,
    ac1_z,
    statesize
};

enum measurement{

    mpc1_x=0,
    mpc1_y,
    mpc1_z,

    mac1_x,
    mac1_y,
    mac1_z,

    measurementsize
};


class forceest : public ukf
{

public:
  lpf2 *lpf;
forceest(int x, int y) : ukf(x,y){
}

Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);
private:


};

#endif // FORCEEST_H
