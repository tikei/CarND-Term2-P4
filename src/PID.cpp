#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;
  p_error = 0;
  d_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error = TotalError();
}

double PID::TotalError() {
  i_error += p_error;
  return i_error;
}

double PID::GetSteering(double cte) {
  return -Kp * cte - Kd * d_error - Ki * i_error;
}

double PID::GetThrottle( double max_throttle) {
  return max_throttle -Kp * p_error - Kd * d_error - Ki * i_error;
}

