#ifndef RLS_H
#define RLS_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class rls
{
public:


   rls();
   void set_regressor();

   Eigen::VectorXd update(double x, double y);



private:
/*
    std::vector<double> x;
    std::vector<double> theta;
*/

 double X_col ,X_raw;
 double theta_col , theta_raw;
 Eigen::MatrixXd Y;        //for measurement
 Eigen::MatrixXd Y_k_1;


 Eigen::MatrixXd X;       // for state
 Eigen::MatrixXd X_k_1;

 Eigen::MatrixXd theta_hat;   // for parameter
 Eigen::MatrixXd theta_hat_k_1;


 Eigen::MatrixXd psi;     //kalman gain
 Eigen::MatrixXd psi_k_1;
 Eigen::MatrixXd P;
 Eigen::MatrixXd P_k_1;



 double lambda;

};

#endif // RLS_H
