#include "mpc_controller_ros2/mpc_torque_solver.hpp"
#include <Eigen/Dense>
#include <cstring>

namespace mpc_controller_ros2 {

MPCTorqueSolver::MPCTorqueSolver(rclcpp::Logger logger) : logger_(logger) {
  nx_ = QUADROTOR_TORQUE_NX;
  nu_ = QUADROTOR_TORQUE_NU;
  ny_ = nx_ + nu_;
  ny_e_ = nx_;

  // Use horizon from generated code
  int n_horizon = getNHorizon();
  x_traj_.resize((n_horizon + 1) * nx_);
  u_traj_.resize(n_horizon * nu_);
}

MPCTorqueSolver::~MPCTorqueSolver() {
  if (capsule_ != nullptr) {
    int status = quadrotor_torque_acados_free(capsule_);
    if (status) {
      RCLCPP_ERROR(logger_, "quadrotor_torque_acados_free() returned status %d",
                   status);
    }
    quadrotor_torque_acados_free_capsule(capsule_);
  }
}

void MPCTorqueSolver::setStageVaryingCostMatrices(
    const std::vector<double>& Q_base_diag,
    const std::vector<double>& R_base_diag,
    double horizon_decay_factor,
    const std::string& decay_type) {

    int n_horizon = getNHorizon();

    for (int i = 0; i < n_horizon; i++) {
        double weight_factor = 1.0;
        double progress = (double)i / n_horizon;

        if (decay_type == "linear") {
            weight_factor = 1.0 - (horizon_decay_factor * progress);
        } else if (decay_type == "exponential") {
            double k = -log(1.0 - horizon_decay_factor);
            weight_factor = exp(-k * progress);
        }

        std::vector<double> W(ny_ * ny_, 0.0);

        // Apply to all 13 states
        for (int j = 0; j < 13; j++) {
            W[j * ny_ + j] = Q_base_diag[j] * weight_factor;
        }

        // Control weights
        W[13 * ny_ + 13] = R_base_diag[0] * weight_factor;  // Thrust
        W[14 * ny_ + 14] = R_base_diag[1] * weight_factor;  // tau_x
        W[15 * ny_ + 15] = R_base_diag[2] * weight_factor;  // tau_y
        W[16 * ny_ + 16] = R_base_diag[3] * weight_factor;  // tau_z

        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W.data());
    }

    // Terminal cost
    double terminal_factor = (decay_type == "exponential") ?
        exp(-log(1.0 - horizon_decay_factor)) : (1.0 - horizon_decay_factor);

    std::vector<double> W_e(nx_ * nx_, 0.0);
    for (int i = 0; i < nx_; i++) {
        W_e[i * nx_ + i] = Q_base_diag[i] * terminal_factor;
    }
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, n_horizon, "W", W_e.data());
}

bool MPCTorqueSolver::initialize() {
    if (initialized_) {
        return true;
    }

    capsule_ = quadrotor_torque_acados_create_capsule();
    int status = quadrotor_torque_acados_create(capsule_);

    if (status) {
        RCLCPP_ERROR(logger_, "Failed to create torque solver, status: %d", status);
        return false;
    }

    nlp_config_ = quadrotor_torque_acados_get_nlp_config(capsule_);
    nlp_dims_ = quadrotor_torque_acados_get_nlp_dims(capsule_);
    nlp_in_ = quadrotor_torque_acados_get_nlp_in(capsule_);
    nlp_out_ = quadrotor_torque_acados_get_nlp_out(capsule_);
    nlp_solver_ = quadrotor_torque_acados_get_nlp_solver(capsule_);

    // Extract time step from solver options
    // The total time horizon (Tf) is stored in the solver options
    void* nlp_opts = quadrotor_torque_acados_get_nlp_opts(capsule_);

    // Calculate time step: dt = Tf / N
    int n_horizon = getNHorizon();
    ocp_nlp_in_get(nlp_config_, nlp_dims_, nlp_in_, 0, "Ts", &time_step_);

    // CRITICAL: Set parameters immediately like test
    double params[8] = {1.47, 0.03, 0.03, 0.05, 0.1, 0.1, 0.1, 0.0}; // mass, Ixx, Iyy, Izz, kx, ky, kz, kh
    for (int i = 0; i <= n_horizon; ++i) {
        quadrotor_torque_acados_update_params(capsule_, i, params, 8);
    }

    // Initialize trajectory EXACTLY like test
    for (int i = 0; i <= n_horizon; ++i) {
        double x_init[QUADROTOR_TORQUE_NX] = {0};
        x_init[6] = 1.0;  // qw
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_init);
        memcpy(&x_traj_[i * nx_], x_init, nx_ * sizeof(double));
    }

    double u_hover = 1.47 * 9.81;
    for (int i = 0; i < n_horizon; ++i) {
        double u_init[QUADROTOR_TORQUE_NU] = {u_hover, 0.0, 0.0, 0.0};
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_init);
        memcpy(&u_traj_[i * nu_], u_init, nu_ * sizeof(double));
    }

    initialized_ = true;
    RCLCPP_INFO(logger_, "Torque solver initialized successfully");
    return true;
}

void MPCTorqueSolver::setParameters(const std::vector<double> &params) {
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Solver not initialized");
    return;
  }

  // params should be [mass, Ixx, Iyy, Izz, kx, ky, kz, kh]
  if (params.size() != 8) {
    RCLCPP_ERROR(logger_,
                 "Invalid parameter size for torque model. Expected 8, got %zu",
                 params.size());
    return;
  }

  double p_params[8];
  std::copy(params.begin(), params.end(), p_params);

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  for (int i = 0; i <= n_horizon; ++i) {
    quadrotor_torque_acados_update_params(capsule_, i, p_params, 8);
  }
}

void MPCTorqueSolver::setCostMatrices(const std::vector<double> &Q_diag,
                                      const std::vector<double> &R_diag) {
  if (!initialized_)
    return;

  // Validate input dimensions
  if (Q_diag.size() != static_cast<size_t>(nx_)) {
    RCLCPP_ERROR(logger_, "Q_diag size mismatch. Expected %d, got %zu", nx_,
                 Q_diag.size());
    RCLCPP_ERROR(logger_, "Please check mpc.Q_torque_diag in your config file");
    return;
  }

  if (R_diag.size() != static_cast<size_t>(nu_)) {
    RCLCPP_ERROR(logger_, "R_diag size mismatch. Expected %d, got %zu", nu_,
                 R_diag.size());
    RCLCPP_ERROR(logger_, "Please check mpc.R_torque_diag in your config file");
    return;
  }

  Eigen::MatrixXd W = Eigen::MatrixXd::Zero(ny_, ny_);
  for (int i = 0; i < nx_; ++i) {
    W(i, i) = Q_diag[i];
  }
  for (int i = 0; i < nu_; ++i) {
    W(nx_ + i, nx_ + i) = R_diag[i];
  }

  Eigen::MatrixXd W_e = Eigen::MatrixXd::Zero(ny_e_, ny_e_);
  for (int i = 0; i < nx_; ++i) {
    W_e(i, i) = Q_diag[i];
  }

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  for (int i = 0; i < n_horizon; ++i) {
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W.data());
  }
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, n_horizon, "W",
                         W_e.data());
}

void MPCTorqueSolver::setReference(int stage, const std::vector<double> &yref) {
  if (!initialized_)
    return;

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  if (stage < 0 || stage >= n_horizon) {
    RCLCPP_ERROR(logger_, "Invalid stage %d", stage);
    return;
  }

  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "yref",
                         const_cast<double *>(yref.data()));
}

void MPCTorqueSolver::setTerminalReference(const std::vector<double> &yref_e) {
  if (!initialized_)
    return;

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, n_horizon, "yref",
                         const_cast<double *>(yref_e.data()));
}

void MPCTorqueSolver::setInitialState(const std::vector<double> &x0) {
  if (!initialized_)
    return;

  // EXACTLY like test_mpc_trajectory: Set both lbx and ubx to x0
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                "ubx", const_cast<double *>(x0.data()));

  // Also update the initial guess - this is CRITICAL
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, 0, "x",
                  const_cast<double *>(x0.data()));
}

void MPCTorqueSolver::setStateBounds(int stage, const std::vector<double> &lbx,
                                     const std::vector<double> &ubx) {
  if (!initialized_)
    return;

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  // For torque model, we typically bound position, velocity, and angular rates
  // The quaternion should not be bounded
  if (stage > 0 && stage <= n_horizon) {
    // Set up Jbx matrix to select which states to bound (pos, vel, angular rates)
    Eigen::MatrixXd Jbx = Eigen::MatrixXd::Zero(9, nx_);
    for (int i = 0; i < 6; ++i) {
      Jbx(i, i) = 1.0; // Position and velocity
    }
    for (int i = 0; i < 3; ++i) {
      Jbx(6 + i, 10 + i) = 1.0; // Angular rates
    }

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  stage, "Jbx", Jbx.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  stage, "lbx",
                                  const_cast<double *>(lbx.data()));
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  stage, "ubx",
                                  const_cast<double *>(ubx.data()));
  }
}

void MPCTorqueSolver::setControlBounds(const std::vector<double> &lbu,
                                       const std::vector<double> &ubu) {
  if (!initialized_)
    return;

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  for (int i = 0; i < n_horizon; ++i) {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i,
                                  "lbu", const_cast<double *>(lbu.data()));
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i,
                                  "ubu", const_cast<double *>(ubu.data()));
  }
}

int MPCTorqueSolver::solve() {
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Solver not initialized");
    return -1;
  }

  return quadrotor_torque_acados_solve(capsule_);
}

void MPCTorqueSolver::getState(int stage, std::vector<double> &x) {
  if (!initialized_)
    return;

  x.resize(nx_);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
}

void MPCTorqueSolver::getControl(int stage, std::vector<double> &u) {
  if (!initialized_)
    return;

  u.resize(nu_);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
}

void MPCTorqueSolver::warmStart(
    const std::vector<std::vector<double>> &x_traj,
    const std::vector<std::vector<double>> &u_traj) {
  if (!initialized_)
    return;

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_
  // Set initial trajectory guess
  for (size_t i = 0; i < x_traj.size() && i <= static_cast<size_t>(n_horizon);
       ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x",
                    const_cast<double *>(x_traj[i].data()));
  }

  for (size_t i = 0; i < u_traj.size() && i < static_cast<size_t>(n_horizon);
       ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u",
                    const_cast<double *>(u_traj[i].data()));
  }
}

void MPCTorqueSolver::reset() {
  if (!initialized_)
    return;

  int n_horizon = getNHorizon();  // Use getter instead of n_horizon_

  // Set a feasible initial guess for entire horizon
  std::vector<double> x_init(nx_, 0.0);
  x_init[6] = 1.0; // qw = 1 for valid quaternion
  // Note: For torque model, angular velocities (states 10,11,12) start at 0

  std::vector<double> u_init(nu_, 0.0);
  u_init[0] = 14.4; // Approximate hover thrust
  // Torques (u[1], u[2], u[3]) start at 0

  for (int i = 0; i <= n_horizon; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x",
                    x_init.data());
  }

  for (int i = 0; i < n_horizon; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u",
                    u_init.data());
  }
}

} // namespace mpc_controller_ros2
