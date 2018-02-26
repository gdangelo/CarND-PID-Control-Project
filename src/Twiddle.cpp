#include "Twiddle.h"

Twiddle::Twiddle(int max_dist) {
  this->is_used = true;
  this->is_initialized = false;

  this->nb_params = 3;

  this->param_index = 0;

  for(unsigned int i = 0; i < this->nb_params; ++i) {
    this->dp.push_back(0.0);
  }

  this->max_dist = max_dist;
  this->dist_count = 0;
}

Twiddle::~Twiddle() {}
