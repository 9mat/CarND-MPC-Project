#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.16;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

#define N_STATES 4
#define N_ERRORS 2
#define N_ACTUATORS 2


int x_start     = 0;
int y_start     = x_start + N;
int psi_start   = y_start + N;
int v_start     = psi_start + N;
int cte_start   = v_start + N;
int epsi_start  = cte_start + N;
int delta_start = epsi_start + N;
int a_start     = delta_start + N - 1;

// Evaluate a polynomial.
template<typename T> 
T polyeval(Eigen::VectorXd coeffs, T x) {
  size_t n = coeffs.size();
  T result = coeffs[n-1];
  for (size_t i = n-2; i >= 0; i--) {
    result *= x; 
    result += coeffs[i];
  }
  return result;
}

Eigen::VectorXd differentiate(const Eigen::VectorXd &coeffs) {
  size_t n = coeffs.size();
  Eigen::VectorXd coeffs_grad(n-1);
  
  for(size_t i=1; i<n; i++){
    coeffs_grad[i-1] = coeffs[i]*i;
  }

  return coeffs_grad;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  Eigen::VectorXd coeffs_grad;
  FG_eval(Eigen::VectorXd coeffs) { 
    this->coeffs = coeffs; 
    coeffs_grad = differentiate(coeffs);
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // cost
    for(size_t t=0; t<N; t++){
      fg[0] += CppAD::pow(vars[cte_start+t],2);
      fg[0] += CppAD::pow(vars[epsi_start+t],2);
    }

    // initialization
    fg[1+x_start]     = vars[x_start];
    fg[1+y_start]     = vars[y_start];
    fg[1+psi_start]   = vars[psi_start];
    fg[1+v_start]     = vars[v_start];
    fg[1+cte_start]   = vars[cte_start];
    fg[1+epsi_start]  = vars[epsi_start];

    // constraints (special)
    fg[1+cte_start]   = vars[cte_start] - (polyeval(coeffs, vars[x_start]) - vars[y_start]);
    fg[1+epsi_start]  = vars[epsi_start] - (vars[psi_start] - CppAD::atan(polyeval(coeffs_grad, vars[x_start])));

    // constraints
    for(size_t t=1; t<N; t++){
      AD<double> x0   = vars[x_start+t-1];
      AD<double> y0   = vars[y_start+t-1];
      AD<double> v0   = vars[v_start+t-1];
      AD<double> psi0 = vars[psi_start+t-1];
      AD<double> delta0 = vars[delta_start+t-1];
      AD<double> a0   = vars[a_start+t-1];

      fg[1+x_start+t]   = vars[x_start+t] - (x0 + v0*CppAD::cos(psi0)*dt);
      fg[1+y_start+t]   = vars[y_start+t] - (y0 + v0*CppAD::sin(psi0)*dt);
      fg[1+psi_start+t] = vars[psi_start+t] - (psi0 + v0/Lf*delta0*dt);
      fg[1+v_start+t]   = vars[v_start+t] - (v0 + a0*dt);

      fg[1+cte_start+t] = vars[cte_start+t] - (polyeval(coeffs,x0) - y0 + v0*CppAD::sin(vars[epsi_start+t-1])*dt);
      fg[1+epsi_start+t] = vars[epsi_start+t] - (vars[psi_start+t] - CppAD::atan(polyeval(coeffs_grad,x0)));
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = (N_STATES + N_ERRORS + N_ACTUATORS)*N - N_ACTUATORS;

  // TODO: Set the number of constraints
  size_t n_constraints = (N_STATES + N_ERRORS)*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  for(size_t t=0; t<N; i++){
    vars_lowerbound[x_start + t] = -1e3;
    vars_upperbound[x_start + t] = 1e3;

    vars_lowerbound[y_start + t] = -1e3;
    vars_upperbound[y_start + t] = 1e3;

    vars_lowerbound[psi_start + t] = -5;
    vars_upperbound[psi_start + t] = 5;    

    vars_lowerbound[v_start + t] = 0;
    vars_upperbound[v_start + t] = 100;

    vars_lowerbound[cte_start + t] = -5;
    vars_upperbound[cte_start + t] = 5;

    vars_lowerbound[epsi_start + t] = -5;
    vars_upperbound[epsi_start + t] = 5;
  }

  for(size_t t=0; t<N-1; t++){
    vars_lowerbound[delta_start + t] = -0.6;
    vars_upperbound[delta_start + t] = 0.6;    

    vars_lowerbound[a_start + t] = -1;
    vars_upperbound[a_start + t] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  double x0 = state[0], y0 = state[1], psi0 = state[2], v0 = state[3];
  double cte0 = polyeval(coeffs, x0) - y0;
  double epsi0 = psi0 - atan(polyeval(differentiate(coeffs), x0));

  cout<<"here"<<endl;
  constraints_lowerbound[x_start]   = constraints_upperbound[x_start]   = x0;
  constraints_lowerbound[y_start]   = constraints_upperbound[y_start]   = y0;
  constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = psi0;
  constraints_lowerbound[v_start]   = constraints_upperbound[v_start]   = v0;
  constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = cte0;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi0;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {solution.x[delta_start], solution.x[a_start]};
}
