#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID()
	:_p_error(0),
	 _i_error(0),
	 _d_error(0) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	_Kp = Kp;
  	_Ki = Ki;
  	_Kd = Kd;

    _p_error = 0;
    _d_error = 0;
    _i_error = 0;
}

void PID::UpdateError(double cte) {
	double current = _p_error;


	_p_error  = cte;
	_d_error  = cte - current;
	_i_error += cte;
}

double PID::TotalError() {
	return -_Kp * _p_error - _Kd * _d_error - _Ki * _i_error; 
}

