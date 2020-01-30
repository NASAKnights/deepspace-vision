#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

PID::PID( double dt, double max, double min, double Kp, double Ki, double Kd ):
  _dt(dt),
  _max(max),
  _min(min),
  _Kp(Kp),
  _Kd(Kd),
  _Ki(Ki),
  _pre_error(0),
  _integral(0),
  oldState(0)
{
}


void PID::button(int buttonState){

  if(buttonState==1 && oldState == 0){
    _integral=0;
    _pre_error=0;
  }
  oldState=buttonState;
}



double PID::calculate( double setpoint, double pv ,double dt, double* P, double* I, double* D)
{
  _dt = dt;
    
  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = _Kp * error;

  // Integral term
  _integral += error * _dt;
  double Iout = _Ki * _integral;

  // Derivative term
  double derivative = (error - _pre_error) / _dt;
  double Dout = _Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;
  if(abs(Pout) > 1.2)
    Pout = copysign(1.2,Pout);
  if(abs(Iout) > 1.2)
    Iout = copysign(1.2,Iout);
  if(abs(Dout) > 1.2)
    Dout = copysign(1.2,Dout);
  printf("fix-gyro:%6.2f, %6.2f, errorPID: %6.2f, Pout: %6.2f, Iout: %6.2f, Dout:%6.2f, output: %6.2f\n",setpoint,pv,error,Pout,Iout,Dout,output);
    
  *P=Pout;
  *I=Iout;
  *D=Dout;
  // Restrict to max/min
  if( output > _max )
    output = _max;
  else if( output < _min )
    output = _min;

  // Save error to previous error
  _pre_error = error;
  return output;
}

#endif
