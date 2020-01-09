#ifndef _PID_H_
#define _PID_H_

//class PIDImpl;
class PID {
public:
  PID( double dt, double max, double min, double Kp, double Ki, double Kd );
  double calculate( double setpoint, double pv ,double dt);

private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
};

#endif
