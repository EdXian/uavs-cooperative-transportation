#include "forceest.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//recovery

Eigen::MatrixXd forceest::  dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);

    for(int i=0;i<this->x_sigmavector_size;i++){
        //sigma points
        double px = sigma_state(p_x,i);
        double py = sigma_state(p_y,i);
        double pz = sigma_state(p_z,i);
        double vx = sigma_state(v_x,i);
        double vy = sigma_state(v_y,i);
        double vz = sigma_state(v_z,i);
        double ex = sigma_state(e_x,i); // delta_p
        double ey = sigma_state(e_y,i);
        double ez = sigma_state(e_z,i);

        double omegax = sigma_state(omega_x,i);
        double omegay = sigma_state(omega_y,i);
        double omegaz = sigma_state(omega_z,i);

        double ax = sigma_state(a_x,i);
        double ay = sigma_state(a_y,i);
        double az = sigma_state(a_z,i);


        double Fx = sigma_state(F_x,i)+ gausian_noise(0)  ;

        double Fy = sigma_state(F_y,i) + gausian_noise(1) ;
        double Fz = sigma_state(F_z,i) + gausian_noise(2) ;
        double tauz = sigma_state(tau_z,i);

        double betax = sigma_state(beta_x,i);
        double betay = sigma_state(beta_y,i);
        double betaz = sigma_state(beta_z,i);

        double a;
        double f;
        double delta_q4;
        Eigen::Vector3d delta_q3;
        Eigen::Vector3d delta_p;
        Eigen::VectorXd delta_q(4);

        Eigen::VectorXd q_k_sigma(4);
        Eigen::VectorXd q_k1_sigma(4);
        Eigen::MatrixXd phi_q(4,4);
        Eigen::MatrixXd omega(4,4);
        Eigen::VectorXd delta_q_k1(4);

        Eigen::MatrixXd phi_q_1(4,4);
        Eigen::Vector3d delta_p_k1;
        double delta_q_4_k1;
        double delta_t;

        Eigen::Vector3d v_k1,v_k;
        Eigen::Vector3d thrust_v;
        Eigen::Vector3d gravity, F_v;
        Eigen::Matrix3d J;
        Eigen::Vector3d omega_v, omega_v1;
        Eigen::Vector3d torque_v;
        Eigen::Vector3d p_v;
        Eigen::Vector3d p_v1;
        Eigen::Vector3d beta;
        double omega_value;
        const double m = 1.50;
        Eigen::Vector3d thrust_test;
        v_k << vx, vy, vz;
        thrust_v << 0, 0, thrust;
        gravity << 0, 0, 9.81;
        F_v << Fx, Fy, Fz;
        J << 0.0625, 0, 0,  //0.0625,0.0625,0.12656
             0, 0.0625, 0,
             0, 0, 0.12656;
        omega_v << omegax, omegay, omegaz;
        torque_v << 0, 0, tauz;
        //beta << betax, betay, betaz;
        //USQUE
        delta_p << ex, ey, ez;
        p_v << px, py, pz;
        a = 3; //1.7
        f = 2*a + 1;
        delta_t = 0.02;


        delta_q4 = (-a*(ex*ex+ey*ey+ez*ez) + f*sqrt(f*f + (1 - a*a)*(ex*ex+ey*ey+ez*ez)))/(f*f + ex*ex+ey*ey+ez*ez);
        delta_q3 = (a+delta_q4)*delta_p/f;
        delta_q << delta_q3, delta_q4;





        //compute sigma-point quaternions from error quaternions, initial q_k = 0,0,0,1
        if( i == 0){
          q_k_sigma = last_quat;
        }
        else
        {
          phi_q.setZero();
          phi_q << delta_q4, -delta_q3(2), delta_q3(1), delta_q3(0),
                 delta_q3(2), delta_q4, -delta_q3(0), delta_q3(1),
                 -delta_q3(1), delta_q3(0), delta_q4, delta_q3(2),
                 -delta_q3(0), -delta_q3(1), -delta_q3(2), delta_q4;
          q_k_sigma = phi_q * last_quat;
        }



        //The quaternions are subsequently propagated forward in time using

        if(omegax == 0){
          omegax = 0.0001;
        }
        if(omegay == 0){
          omegay = 0.0001;
        }
        if(omegaz ==0){
          omegaz = 0.0001;
        }

        omega_value = omegax*omegax+omegay*omegay+omegaz*omegaz;
        omega << cos(0.5*sqrt(omega_value)*delta_t), sin(0.5*sqrt(omega_value)*delta_t)*omegaz/(sqrt(omega_value)), -sin(0.5*sqrt(omega_value)*delta_t)*omegay/(sqrt(omega_value)), sin(0.5*sqrt(omega_value)*delta_t)*omegax/(sqrt(omega_value)),
                 -sin(0.5*sqrt(omega_value)*delta_t)*omegaz/(sqrt(omega_value)), cos(0.5*sqrt(omega_value)*delta_t), sin(0.5*sqrt(omega_value)*delta_t)*omegax/(sqrt(omega_value)), sin(0.5*sqrt(omega_value)*delta_t)*omegay/(sqrt(omega_value)),
                 sin(0.5*sqrt(omega_value)*delta_t)*omegay/(sqrt(omega_value)), -sin(0.5*sqrt(omega_value)*delta_t)*omegax/(sqrt(omega_value)), cos(0.5*sqrt(omega_value)*delta_t), sin(0.5*sqrt(omega_value)*delta_t)*omegaz/(sqrt(omega_value)),
                -sin(0.5*sqrt(omega_value)*delta_t)*omegax/(sqrt(omega_value)), -sin(0.5*sqrt(omega_value)*delta_t)*omegay/(sqrt(omega_value)), -sin(0.5*sqrt(omega_value)*delta_t)*omegaz/(sqrt(omega_value)), cos(0.5*sqrt(omega_value)*delta_t);
        q_k1_sigma = omega * q_k_sigma;



        if(i==0){
          qk1 = q_k1_sigma; // 第0個sigma point,會丟進去correct

        }
        //the propagated error quaternions are determined usingm, quater是否恆等於1
        double quater;

        if(i == 0){
          q_k1_0.setZero();
          quater = q_k1_sigma(0) * q_k1_sigma(0) + q_k1_sigma(1) * q_k1_sigma(1) + q_k1_sigma(2) * q_k1_sigma(2) + q_k1_sigma(3) * q_k1_sigma(3);
          if(quater == 0){
            quater = 1;
          }

          q_k1_0 << -q_k1_sigma(0), -q_k1_sigma(1), -q_k1_sigma(2), q_k1_sigma(3);

          phi_q_1 << q_k1_sigma(3), -q_k1_sigma(2), q_k1_sigma(1), q_k1_sigma(0),
                    q_k1_sigma(2), q_k1_sigma(3), -q_k1_sigma(0), q_k1_sigma(1),
                    -q_k1_sigma(1), q_k1_sigma(0), q_k1_sigma(3), q_k1_sigma(2),
                    -q_k1_sigma(0), -q_k1_sigma(1), -q_k1_sigma(2), q_k1_sigma(3);

          delta_q_k1 = phi_q_1 * q_k1_0;
        }
        else{

          phi_q_1 << q_k1_sigma(3), -q_k1_sigma(2), q_k1_sigma(1), q_k1_sigma(0),
                    q_k1_sigma(2), q_k1_sigma(3), -q_k1_sigma(0), q_k1_sigma(1),
                    -q_k1_sigma(1), q_k1_sigma(0), q_k1_sigma(3), q_k1_sigma(2),
                    -q_k1_sigma(0), -q_k1_sigma(1), -q_k1_sigma(2), q_k1_sigma(3);
          delta_q_k1 = phi_q_1 * q_k1_0;
        }




        //he propagated sigma points are calculated using
        delta_p_k1 << delta_q_k1(0), delta_q_k1(1), delta_q_k1(2);
        delta_q_4_k1 = delta_q_k1(3);

        predict_sigma_state(e_x,i) = f*delta_p_k1(0)/(a + delta_q_4_k1);
        predict_sigma_state(e_y,i) = f*delta_p_k1(1)/(a + delta_q_4_k1);
        predict_sigma_state(e_z,i) = f*delta_p_k1(2)/(a + delta_q_4_k1);




        if(i == 0){
          predict_sigma_state(e_x,i) = 0;
          predict_sigma_state(e_y,i) = 0;
          predict_sigma_state(e_z,i) = 0;
        }



        //position

        p_v1 = p_v + delta_t * v_k;

        //velocity

        Eigen::Vector3d pseudo_a =  ((R_IB*(thrust_v))/m - gravity + F_v/m);
        Eigen::Vector3d pseudo_b;
        pseudo_b <<a_x,a_y,a_z;
        double alpha=0.00;
        v_k1 = v_k + delta_t * ( (1-alpha) *pseudo_a+ (alpha)*pseudo_b  )  ;

        //rotation

        omega_v1 = omega_v + delta_t * J.inverse()*(U + torque_v - omega_v.cross(J*omega_v));

        //






        predict_sigma_state(p_x,i) = p_v1(0);
        predict_sigma_state(p_y,i) = p_v1(1);
        predict_sigma_state(p_z,i) = p_v1(2);

        predict_sigma_state(F_x,i) = Fx;
        predict_sigma_state(F_y,i) = Fy;
        predict_sigma_state(F_z,i) = Fz;
        predict_sigma_state(tau_z,i) = tauz;

        predict_sigma_state(v_x,i) = v_k1(0);
        predict_sigma_state(v_y,i) = v_k1(1);
        predict_sigma_state(v_z,i) = v_k1(2);

        predict_sigma_state(omega_x,i) = omega_v1(0);
        predict_sigma_state(omega_y,i) = omega_v1(1);
        predict_sigma_state(omega_z,i) = omega_v1(2);


        predict_sigma_state(beta_x,i) = betax;
        predict_sigma_state(beta_y,i) = betay;
        predict_sigma_state(beta_z,i) = betaz;

        predict_sigma_state(a_x,i) = a_x;
        predict_sigma_state(a_y,i) = a_y;
        predict_sigma_state(a_z,i) = a_z;


    }
    return predict_sigma_state;


}

Eigen::MatrixXd forceest::state_to_measure(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_measure(this->y_size,this->x_sigmavector_size);

    for(int i=0;i<this->x_sigmavector_size;i++){

        predict_sigma_measure( mp_x ,i) =   sigma_state(p_x,i);
        predict_sigma_measure( mp_y ,i) =   sigma_state(p_y,i);
        predict_sigma_measure( mp_z ,i) =   sigma_state(p_z,i);

        predict_sigma_measure( mv_x ,i) =   sigma_state(v_x,i);
        predict_sigma_measure( mv_y ,i) =   sigma_state(v_y,i);
        predict_sigma_measure( mv_z ,i) =   sigma_state(v_z,i);

        predict_sigma_measure( me_x ,i) =   sigma_state(e_x,i);
        predict_sigma_measure( me_y ,i) =   sigma_state(e_y,i);
        predict_sigma_measure( me_z ,i) =   sigma_state(e_z,i);


        predict_sigma_measure( momega_x ,i) =   sigma_state(omega_x,i);
        predict_sigma_measure( momega_y ,i) =   sigma_state(omega_y,i);
        predict_sigma_measure( momega_z ,i) =   sigma_state(omega_z,i);

        predict_sigma_measure( ma_x ,i) =   sigma_state(a_x,i);
        predict_sigma_measure( ma_y ,i) =   sigma_state(a_y,i);
        predict_sigma_measure( ma_z ,i) =   sigma_state(a_z,i);

/*
        predict_sigma_measure( mq_x ,i) =   sigma_state(q_x,i);
        predict_sigma_measure( mq_y ,i) =   sigma_state(q_y,i);
        predict_sigma_measure( mq_z ,i) =   sigma_state(q_z,i);
*/
    }
    return predict_sigma_measure;


}

