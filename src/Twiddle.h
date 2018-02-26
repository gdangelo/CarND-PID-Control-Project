#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include "PID.h"

using namespace std;

enum DIRECTION {
  FORWARD,
  BACKWARD
};

struct update {
  double value;
  DIRECTION direction;
};

class Twiddle {
public:

  ///* initially set to true, set to false once parameters optimization is done
  bool is_used;

  ///* initially set to false, set to true in first simulator run
  bool is_initialized;

  ///* number of PID parameters (3 by default)
  int nb_params;

  ///* distance already run on the current loop
  int dist_count;

  ///* maximum distance to run each time the simulator is run
  int max_dist;

  ///* tolerance threshold
  double tolerance;

  ///* best error
  double best_error;

  ///* error on the current run
  double error;

  ///* index of the current parameters to optimize (p: 0, i: 1, d: 2)
  int param_index;

  ///* values used to update each PID parameters in Twiddle algorithm
  std::vector<update> dp;

  ///* iteration number
  int it;

  /*
  * Constructor
  */
  Twiddle(int max_dist, double tolerance);

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  void Init(PID pid);

  bool DistanceReached();

  double SumDP();
};

#endif /* TWIDDLE_H */
