#ifndef _PID_H_
#define _PID_H_

class PID {
public:
  PID( double dt, double max, double min, double Kp, double Ki, double Kd );
  double calculate( double setpoint, double pv ,double dt,double* P, double* I, double* D);
  void button(int buttonState);
private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  int error_pointer;
  double _pre_error;
  double _integral;
  int oldState;
  static const int arrLen = 100;
  double error_arr[arrLen];
};

#endif
