#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

using namespace std;

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

  ///* best CTE on the current run
  double best_err;

  ///* index of the current parameters to optimize (p: 0, i: 1, d: 2)
  int param_index;

  ///* values used to update each PID parameters in Twiddle algorithm
  std::vector<double> dp;

  /*
  * Constructor
  */
  Twiddle(int max_dist);

  /*
  * Destructor.
  */
  virtual ~Twiddle();
};

#endif /* TWIDDLE_H */
