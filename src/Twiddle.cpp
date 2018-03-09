#include "Twiddle.h"

#include <iostream>
#include <math.h>

Twiddle::Twiddle(int max_dist) {
  this->is_used = max_dist == -1 ? false : true;
  this->is_initialized = false;
  this->it = 0;
  // Twiddle parameters
  this->nb_params = 5;
  this->param_index = 0;
  // Distance
  this->max_dist = max_dist;
  this->dist_count = 0;
  this->best_dist = 0;
  // Error (~cte)
  this->error = 0.0;
  this->avg_error = 0.0;
  this->best_error = INFINITY;

  // Initialize dp parameters
  for (int i = 0; i < this->nb_params; i++) {
    dp_state init = { 1.0, DIRECTION::FORWARD };
    this->dp.push_back(init);
  }
}

Twiddle::~Twiddle() {}

void Twiddle::Init(PID &pid) {
  // Set best error
  best_error = avg_error;
  // Set best dist
  best_dist = dist_count;
  // Initialization is done!
  is_initialized = true;
}

void Twiddle::UpdateBestError() {
  // Set current error as the best one
  best_error = avg_error;
  // Set current distance count as the best one
  best_dist = dist_count;
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
  param_index = (param_index + 1) % nb_params;
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

double Twiddle::SumDp() {
  double sum = 0.0;
  for (int i = 0; i < nb_params; i++) {
    sum += dp[i].value;
  }
  return sum;
}

void Twiddle::PrintStepState(PID &pid) {
  std::cout << "p: ("
            << pid.Kp << ", "
            << pid.Ki << ", "
            << pid.Kd << "), ";

  std::cout << "dp: ("
            << dp[0].value << ", "
            << dp[1].value << ", "
            << dp[2].value << "), ";

  std::cout << "avg err: " << avg_error << ", ";

  std::cout << "dist: " << dist_count << std::endl;
}

void Twiddle::PrintIterationState(PID &pid) {
  std::cout << "Iteration " << it++
            << ", best error: " << best_error
            << ", best dist: "  << best_dist
            << " --> "
            << pid.Kp << "(Kp), "
            << pid.Ki << "(Ki), "
            << pid.Kd << "(Kd)\n"
            << std::endl;
}
