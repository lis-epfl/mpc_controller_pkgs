#ifndef MPC_CONTROLLER_ROS2_MPC_SOLVER_BASE_HPP_
#define MPC_CONTROLLER_ROS2_MPC_SOLVER_BASE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace mpc_controller_ros2 {

class MPCSolverBase {
public:
  virtual ~MPCSolverBase() = default;

  // Pure virtual methods that must be implemented by derived classes
  virtual int getNx() const = 0;
  virtual int getNu() const = 0;
  virtual int getNy() const = 0;
  virtual int getNyE() const = 0;
  virtual int getNHorizon() const = 0;  // Added to get horizon from solver
  virtual double getTimeStep() const = 0;  // Added to get dt from solver

  virtual bool initialize() = 0;
  virtual void setParameters(const std::vector<double> &params) = 0;
  virtual void setCostMatrices(const std::vector<double> &Q_diag,
                               const std::vector<double> &R_diag) = 0;
  virtual void setReference(int stage, const std::vector<double> &yref) = 0;
  virtual void setTerminalReference(const std::vector<double> &yref_e) = 0;
  virtual void setInitialState(const std::vector<double> &x0) = 0;
  virtual void setStateBounds(int stage, const std::vector<double> &lbx,
                              const std::vector<double> &ubx) = 0;
  virtual void
  setStageVaryingCostMatrices(const std::vector<double> &Q_base_diag,
                              const std::vector<double> &R_base_diag,
                              double horizon_decay_factor,
                              const std::string &decay_type) = 0;
  virtual int solve() = 0;
  virtual void getState(int stage, std::vector<double> &x) = 0;
  virtual void getControl(int stage, std::vector<double> &u) = 0;
  virtual void warmStart(const std::vector<std::vector<double>> &x_traj,
                         const std::vector<std::vector<double>> &u_traj) = 0;
  virtual void reset() = 0;

  // Common members - removed n_horizon_ as it will come from derived classes

protected:
  int nx_, nu_, ny_, ny_e_;
};

} // namespace mpc_controller_ros2

#endif // MPC_CONTROLLER_ROS2_MPC_SOLVER_BASE_HPP_
