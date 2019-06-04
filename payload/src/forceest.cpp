#include "forceest.h"

#define L 1.0     //length of object
#define pi 3.14159 //pi
#define g  9.81    //gravity
#define r  0.75   //0.5length of object
#define a 0.01   //width of object
#define l  0.4   //  length of cable
#define M  (0.5)  //Mass of object
#define I  (M*(a*a+L*L))/12 //inertial


Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);
    for(int i=0;i<this->x_sigmavector_size;i++){

        double     p_c1_x     =sigma_state(pc1_x,i);
        double     p_c1_y    =sigma_state(pc1_y,i);
        double     p_c1_z     =sigma_state(pc1_z,i);

        double     v_c1_x     =sigma_state(vc1_x,i);
        double     v_c1_y    =sigma_state(vc1_y,i);
        double     v_c1_z     =sigma_state(vc1_z,i);

        double     a_c1_x     =sigma_state(ac1_x,i);
        double     a_c1_y    =sigma_state(ac1_y,i);
        double     a_c1_z     =sigma_state(ac1_z,i);

        dt=0.02;
        //use global frame
        p_c1_x = p_c1_x + v_c1_x *dt;
        v_c1_x = v_c1_x + a_c1_x *dt;

        p_c1_y = p_c1_y + v_c1_y *dt;
        v_c1_y = v_c1_y + a_c1_y *dt;

        p_c1_z = p_c1_z + v_c1_z *dt;
        v_c1_z = v_c1_z + a_c1_z *dt;


        predict_sigma_state(pc1_x,i) = p_c1_x;
        predict_sigma_state(pc1_y,i) = p_c1_y;
        predict_sigma_state(pc1_z,i) = p_c1_z;

        predict_sigma_state(vc1_x,i) = v_c1_x;
        predict_sigma_state(vc1_y,i) = v_c1_y;
        predict_sigma_state(vc1_z,i) = v_c1_z;

        predict_sigma_state(ac1_x,i) = a_c1_x;
        predict_sigma_state(ac1_y,i) = a_c1_y;
        predict_sigma_state(ac1_z,i) = a_c1_z;


    }
    return predict_sigma_state;
}

