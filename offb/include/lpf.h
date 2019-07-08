#ifndef LPF_H
#define LPF_H
#include "math.h"

class lpf
{
public:
  lpf(double freq , double samplingtime);
  double filter(double state);
  double cutoff_freq;
  double omega ;
  double Ts;
  double last_state;
  double coeff;
  double y_n;
  double y_n1;
  double x;
  bool init ;
private:

};

#endif // LPF_H
