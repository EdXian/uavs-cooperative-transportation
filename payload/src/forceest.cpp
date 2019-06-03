#include "forceest.h"

#define L 1.2     //length of object
#define pi 3.14159 //pi
#define g  9.81    //gravity
#define r  0.75   //0.5length of object
#define a 0.008   //width of object
#define l  0.4   //  length of cable
#define M  (0.3)  //Mass of object
#define I  (M*(a*a+L*L))/12 //inertial

//#define weight 0.45    //weight of object


Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);
    for(int i=0;i<this->x_sigmavector_size;i++){

        double alphar = sigma_state(alpha_r,i);
        double omegar = sigma_state(omega_r,i);
        double alphap = sigma_state(alpha_p,i);
        double omegap = sigma_state(omega_p,i);
        double alphay = sigma_state(alpha_y,i);
        double omegay = sigma_state(omega_y,i);

        double rpx = sigma_state(rp_x,i);
        double rpy = sigma_state(rp_y,i);
        double rpz = sigma_state(rp_z,i);

        double FFx = sigma_state(FF_x,i);
        double FFy = sigma_state(FF_y,i);
        double FFz = sigma_state(FF_z,i);

        double FLx = sigma_state(FL_x,i);
        double FLy = sigma_state(FL_y,i);
        double FLz = sigma_state(FL_z,i);

        double acx = sigma_state(ac_x,i);
        double acz = sigma_state(ac_z,i);
        double acy = sigma_state(ac_y,i);
        double apx = sigma_state(ap_x,i);
        double apy = sigma_state(ap_y,i);
        double apz = sigma_state(ap_z,i);

        Eigen::Vector3d ap , ac  , rp , w , alpha , FF ,FL , Mg ;

        //ap<< apx,apy,apz;
        ac<< acx,acy,acz;
        rp<< rpx, rpy,rpz;
        w << omegar ,omegap , omegay;
        alpha << alphar , alphap , alphay;
        FF << FFx , FFy ,FFz;
        FL << FLx , FLy ,FLz;
        Mg << 0,0,M*g;

        ap = ac - w.cross(w.cross(rp)) - alpha.cross(rp);

        FL = M*ap -FF +Mg;


        predict_sigma_state(rp_x,i) = rpx;
        predict_sigma_state(rp_y,i) = rpy;
        predict_sigma_state(rp_z,i) = rpz;

        predict_sigma_state(omega_r,i) =  omegar;
        predict_sigma_state(omega_p,i) =  omegap;
        predict_sigma_state(omega_y,i) =  omegay;

        predict_sigma_state(alpha_r,i) =  alphar;
        predict_sigma_state(alpha_p,i) =  alphap;
        predict_sigma_state(alpha_y,i) =  alphay;

        predict_sigma_state(FF_x,i) =  FF[0];
        predict_sigma_state(FF_y,i) =  FF[1];
        predict_sigma_state(FF_z,i) =  FF[2];

        predict_sigma_state(FL_x,i) =  FL[0];
        predict_sigma_state(FL_y,i) =  FL[1];
        predict_sigma_state(FL_z,i) =  FL[2];

        predict_sigma_state(ap_x,i) =  ap[0];
        predict_sigma_state(ap_y,i) =  ap[1];
        predict_sigma_state(ap_z,i) =  ap[2];

        predict_sigma_state(ac_x,i) =  ac[0];
        predict_sigma_state(ac_y,i) =  ac[1];
        predict_sigma_state(ac_z,i) =  ac[2];


//        double alpha_p = (r/I)*(FF_net*1-FL_net*1);

////        std::cout << "-----------" <<std::endl;
////        std::cout << "alphap"<<alpha_p<<std::endl;
//        //+alpha_p*r*cos(theta_p)
//        //+alpha_p*r*sin(theta_p)
//       // double alpha_p = (r/I)*(FF_net-FL_net*sin(theta_c+theta_d));

//       // std::cout << "alpha_p" <<  alpha_p <<std::endl;

//        //change to 3D
        
//        predict_sigma_state(thetap,i) = theta_p+omega_p*dt;
//        predict_sigma_state(omegap,i) =  omega_p;

//        predict_sigma_state(ac_x,i) =  acx;
//        predict_sigma_state(ac_z,i) =  acz;

//        predict_sigma_state(FF_x,i) = FFx ;
//        predict_sigma_state(FF_z,i) = FFz ;



//        predict_sigma_state(ap_x,i) = acx - alpha*r*sin(theta_p) - omega_p*omega_p*r*cos(theta_p);
//        predict_sigma_state(ap_z,i) = acz - alpha*r*cos(theta_p) + omega_p*omega_p*r*sin(theta_p);

//        predict_sigma_state(FL_x,i) = M*apx - FFx;
//        predict_sigma_state(FL_z,i) = M*apz + M*g - FFz;



    }
    return predict_sigma_state;
}

