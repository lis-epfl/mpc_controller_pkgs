#ifndef MPC_CONTROLLER_ROS2_MPC_TORQUE_SOLVER_HPP_
#define MPC_CONTROLLER_ROS2_MPC_TORQUE_SOLVER_HPP_

#include "mpc_controller_ros2/mpc_solver_base.hpp"
#include <rclcpp/rclcpp.hpp>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_quadrotor_torque.h"
}

namespace mpc_controller_ros2 {

class MPCTorqueSolver : public MPCSolverBase {
public:
  MPCTorqueSolver(rclcpp::Logger logger);
  ~MPCTorqueSolver();

  // Implement pure virtual methods
  int getNx() const override { return QUADROTOR_TORQUE_NX; }
  int getNu() const override { return QUADROTOR_TORQUE_NU; }
  int getNy() const override {
    return QUADROTOR_TORQUE_NX + QUADROTOR_TORQUE_NU;
  }
  int getNyE() const override { return QUADROTOR_TORQUE_NX; }
  int getNHorizon() const override { return QUADROTOR_TORQUE_N; }  // Get from generated code
  double getTimeStep() const override { return time_step_; }  // Will be extracted from solver

  bool initialize() override;
  void setParameters(const std::vector<double> &params) override;
  void setCostMatrices(const std::vector<double> &Q_diag,
                       const std::vector<double> &R_diag) override;
  void setReference(int stage, const std::vector<double> &yref) override;
  void setTerminalReference(const std::vector<double> &yref_e) override;
  void setInitialState(const std::vector<double> &x0) override;
  void setStateBounds(int stage, const std::vector<double> &lbx,
                      const std::vector<double> &ubx) override;
  void setStageVaryingCostMatrices(const std::vector<double> &Q_base_diag,
                                   const std::vector<double> &R_base_diag,
                                   double horizon_decay_factor,
                                   const std::string &decay_type) override;
  int solve() override;
  void getState(int stage, std::vector<double> &x) override;
  void getControl(int stage, std::vector<double> &u) override;
  void warmStart(const std::vector<std::vector<double>> &x_traj,
                 const std::vector<std::vector<double>> &u_traj) override;
  void reset() override;

  // Torque-specific methods
  void setControlBounds(const std::vector<double> &lbu,
                        const std::vector<double> &ubu);

  // Make internal structures accessible (like in test)
  quadrotor_torque_solver_capsule *getCapsule() { return capsule_; }

  std::vector<double> x_traj_;
  std::vector<double> u_traj_;

  quadrotor_torque_solver_capsule *capsule_ = nullptr;
  ocp_nlp_config *nlp_config_ = nullptr;
  ocp_nlp_dims *nlp_dims_ = nullptr;
  ocp_nlp_in *nlp_in_ = nullptr;
  ocp_nlp_out *nlp_out_ = nullptr;
  ocp_nlp_solver *nlp_solver_ = nullptr;

private:
  rclcpp::Logger logger_;
  bool initialized_ = false;
  double time_step_ = 0.05;  // Will be extracted from solver during initialization

  // Friend class for direct access if needed
  friend class MpcController;
};

} // namespace mpc_controller_ros2

#endif // MPC_CONTROLLER_ROS2_MPC_TORQUE_SOLVER_HPP_
