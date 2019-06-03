#include "rls.h"



rls::rls()
{

   // P=X.transpose()*X;
    //initialize the P matrix

    P = 1e6 * Eigen::MatrixXd::Identity(3, 3);
    P_k_1 =  Eigen::MatrixXd::Identity(3, 3);
    theta_hat = 0.01* theta_hat.setOnes(3,1);
    theta_hat_k_1 =theta_hat.setOnes(3,1);
    X_k_1.setZero(1,3);
    X.setZero(1,3);
    psi_k_1.setZero(3,1);
    psi.setZero(3,1);
    Y_k_1.setZero(1,1);




    std::cout << " P "  << std::endl   << P  <<std::endl;
    std::cout << " theta_hat "  << std::endl   << theta_hat  <<std::endl;
    std::cout << "=================================rls start==========================" <<std::endl;
}


void rls::set_regressor(){

}


//return 0.5*x*x + 0.2*x + 0.3  + (rand()%100/frac);
Eigen::VectorXd rls::update(double x, double y){

    Eigen::MatrixXd lambda = Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd H;//

    X_k_1 << x*x, x ,1;
    H= X_k_1.transpose();

    Y_k_1 << y;

//    std::cout << "X_k_1" << std::endl << X_k_1 << std::endl;
//    std::cout << "H" << std::endl << H << std::endl;
//    std::cout << "Y_k_1" << std::endl << Y_k_1 << std::endl;
//     std::cout << "psi_k_1" << std::endl <<psi_k_1 << std::endl;
    //Algorithm of the RLS
    double den =(H.transpose() * P*H)(0,0) ;
    psi_k_1 = (P*H) * (lambda + H.transpose()*P*H).inverse();
    P_k_1 = P - ((P * H * H.transpose() *P) /(den +0.99)  );
    theta_hat_k_1 = theta_hat + psi_k_1*(Y_k_1 - H.transpose()*theta_hat);

   //update
   theta_hat = theta_hat_k_1;
   P = P_k_1;


   return theta_hat;

}





