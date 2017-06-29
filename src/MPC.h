#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"


using namespace std;
const int N = 10;
const float dt = 0.1;
const int lag_N = 1;

struct mpc_solution {

    vector<double> x;
    vector<double> y;
    vector<double> delta;
    vector<double> a;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  mpc_solution solution_struct_;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
