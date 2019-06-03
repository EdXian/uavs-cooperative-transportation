#include "lpf2.h"
#define pi 3.14159
//reference   http://controlsystemsacademy.com/0020/0020.html
lpf2::lpf2(double freq , double samplingtime) :
  cutoff_freq(freq),
  Ts(samplingtime)
{
  omega = 2*pi*cutoff_freq*samplingtime;
  alpha = tan(omega/2);
  a1 = 2*(alpha*alpha - 1)/(1 + sqrt(2)*alpha + alpha*alpha);
  a2 = (1 - sqrt(2) * alpha + alpha*alpha) / (1 + sqrt(2) * alpha + alpha*alpha);
  b1 = 2;
  b2 = 1;
  k = alpha*alpha/(1+sqrt(2)*alpha+alpha*alpha);
  coeff = exp(-1*omega*Ts);
  init =true;
}

double lpf2::filter(double state){

  if(init){
    x_n = 0;
    x_n1 = 0;
    x_n2 = state;
    y_n = 0;
    y_n1 = x_n2;
    init = false;
  }else{
    x_n2 = state;
    y_n2 = k*x_n2 + k*b1*x_n1 + k*b2*x_n - a1*y_n1 - a2*y_n;
    y_n = y_n1;
    y_n1 = y_n2;
    x_n = x_n1;
    x_n1 = x_n2;
  }
 return y_n2;

}
