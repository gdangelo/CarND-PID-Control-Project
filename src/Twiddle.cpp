#include "Twiddle.h"

Twiddle::Twiddle(int max_dist, double tolerance) {
  this->is_used = true;
  this->is_initialized = false;

  this->nb_params = 3;

  this->param_index = 0;

  for(unsigned int i = 0; i < this->nb_params; ++i) {
    update temp = { 1.0, DIRECTION::FORWARD };
    this->dp.push_back(temp);
  }

  this->tolerance = tolerance;

  this->max_dist = max_dist;
  this->dist_count = 0;

  this->error = 0.0;
}

Twiddle::~Twiddle() {}

void Twiddle::Init(PID &pid) {
  // Set best error
  best_error = error;
  // Use first PID parameter for update
  pid.Kp += dp[param_index].value;
  // Initialization is done!
  is_initialized = true;
}

void Twiddle::UpdateBestError() {
  // Set current error as the best one
  best_error = error;
  // Increase the PID parameter change
  dp[param_index].value *= 1.1;
  // Reset direction to forward
  dp[param_index].direction = DIRECTION::FORWARD;
}

void Twiddle::GoBackward(PID &pid) {
  // Change direction (fwd --> bwd) for parameter optimization
  // by substracting the change 2 times
  switch(param_index) {
    case 0: pid.Kp -= 2*dp[param_index].value; break;
    case 1: pid.Ki -= 2*dp[param_index].value; break;
    case 2: pid.Kd -= 2*dp[param_index].value; break;
  }
  dp[param_index].direction = DIRECTION::BACKWARD;
}

void Twiddle::ChangePIDIndex() {
  // Need 'modulo' operator because the simulator is running indefinitely
  param_index = (param_index + 1) % 3;
}

void Twiddle::UpdatePIDParameter(PID &pid) {
  // Add change to current PID parameter
  switch(param_index) {
    case 0: pid.Kp += dp[param_index].value; break;
    case 1: pid.Ki += dp[param_index].value; break;
    case 2: pid.Kd += dp[param_index].value; break;
  }
}

void Twiddle::ResetPIDParameter(PID &pid) {
  UpdatePIDParameter(pid);
  // Decrease the PID parameter change
  dp[param_index].value *= 0.9;
  // Reset direction to forward
  dp[param_index].direction = DIRECTION::FORWARD;
}

bool Twiddle::DistanceReached() {
  return dist_count >= max_dist;
}
