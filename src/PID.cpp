#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;
}

void PID::UpdateError(double cte) {
  // differential: current cte - previous cte
  d_error = cte - p_error;
  // proportionnal: current cte
  p_error = cte;
  // integral: sum of cte
  i_error += cte;

  // Handle integral windup problem by setting output limits
  if (i_error > max_output_limit) {
    i_error = max_output_limit;
  }
  if (i_error < min_output_limit) {
    i_error = min_output_limit;
  }
}

double PID::TotalError() {
  double total_error = Kp*p_error + Ki*i_error + Kd*d_error;

  // Handle windup problem by setting output limits
  if (total_error > max_output_limit) {
    total_error = max_output_limit;
  }
  if (total_error < min_output_limit) {
    total_error = min_output_limit;
  }

  return total_error;
}
