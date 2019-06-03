#ifndef LPF2_H
#define LPF2_H
#include "math.h"

class lpf2
{
public:
  lpf2(double freq , double samplingtime);
  double filter(double state);
  double cutoff_freq;
  double omega ;
  double Ts;
  double last_state;
  double coeff;
  double alpha;
  double k;
  double b1;
  double b2;
  double a1;
  double a2;
  double y_n;
  double y_n1;
  double y_n2;
  double x_n;
  double x_n1;
  double x_n2;
  bool init ;
private:

};

#endif // LPF_H
