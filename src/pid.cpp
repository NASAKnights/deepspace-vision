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
  _integral(0)
{
}

double PID::calculate( double setpoint, double pv ,double dt)
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

  printf("fix-gyro:%6.2f, %6.2f, errorPID: %6.2f, Pout: %6.2f, Iout: %6.2f, Dout:%6.2f, output: %6.2f",setpoint,pv,error,Pout,Iout,Dout,output);
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
