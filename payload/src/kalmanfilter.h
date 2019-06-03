#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
using namespace Eigen;
class kalmanfilter
{

public:


  int n;
  int m;


  MatrixXf A;
  MatrixXf B;
  MatrixXf H;
  MatrixXf Q;
  MatrixXf R;
  MatrixXf I;

  VectorXf X;
  MatrixXf P;
  MatrixXf K;


  VectorXf X0;
  MatrixXf P0;

  kalmanfilter(int _n, int _m);

  /* Set Fixed Matrix (NO INPUT) */
  void setFixed(MatrixXf _A, MatrixXf _H, MatrixXf _Q, MatrixXf _R);

  /* Set Fixed Matrix (WITH INPUT) */
  //void setFixed(MatrixXf _A, MatrixXf _B, MatrixXf _H, MatrixXf _Q, MatrixXf _R);

  /* Set Initial Value */
  void setInitial(VectorXf _X0, MatrixXf _P0);

  /* Do prediction (NO INPUT) */
  void predict(void);

  /* Do prediction (INPUT) */
  void predict(VectorXf U);

  /* Do correction */
  void correct(VectorXf Z);

};

#endif // KALMANFILTER_H
