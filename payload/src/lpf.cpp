#include "lpf.h"
#define pi 3.14159
//reference   http://controlsystemsacademy.com/0020/0020.html
lpf::lpf(double freq , double samplingtime) :
  cutoff_freq(freq),
  Ts(samplingtime)
{
  omega = 2*pi*cutoff_freq;
  coeff = exp(-1*omega*Ts);
  init =true;
}

double lpf::filter(double state){

  if(init){
    x = state;
    y_n = x;
    init = false;
  }else{
    x = state;
    y_n1 = (coeff) * y_n + (1-coeff) *x;
    y_n = y_n1;
  }
 return y_n1;

}
