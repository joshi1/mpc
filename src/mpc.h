#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

extern bool debugging_enabled;

#define DEBUG(x) do {		        \
    if (debugging_enabled)              \
      { std::cerr << x << std::endl; }	\
} while (0)

extern const double Lf;
extern double radius1;
extern double radius2;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
