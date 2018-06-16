#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// This is the length from front to centre of gravity
const double Lf = 2.67;

struct actuation_vars {
  double d;
  double a;
  vector<double> x_vals;
  vector<double> y_vals;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  actuation_vars Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  // Pass weights to MPC provided on command line
  std::vector<double> weights;
  void pass_args(std::vector<double> args);

};

#endif /* MPC_H */
