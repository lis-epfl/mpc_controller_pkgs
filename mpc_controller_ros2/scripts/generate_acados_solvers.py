#!/usr/bin/env python3
import os
import sys
import shutil
from pathlib import Path
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat, sin, cos, horzcat, cross, inv, diag
import numpy as np
import scipy.linalg

def get_package_root():
    """Get the root directory of the ROS2 package"""
    # This script should be in scripts/ directory
    script_dir = Path(__file__).parent.absolute()

    # Check if we're in scripts directory
    if script_dir.name == 'scripts':
        return script_dir.parent
    else:
        # If run from package root or elsewhere
        return script_dir

def cleanup_generated_code(model_name, package_root):
    """Removes previously generated solver files."""
    dir_to_remove = package_root / 'c_generated_code' / model_name
    if dir_to_remove.exists():
        shutil.rmtree(dir_to_remove)
        print(f"Cleaned up: {dir_to_remove}")

def create_rate_model() -> AcadosModel:
    """Creates the quadrotor rate model - EXACTLY matching test_dynamics.py"""
    model_name = 'quadrotor_rate'

    # Define states
    px, py, pz = SX.sym('px'), SX.sym('py'), SX.sym('pz')
    vx, vy, vz = SX.sym('vx'), SX.sym('vy'), SX.sym('vz')
    qw, qx, qy, qz = SX.sym('qw'), SX.sym('qx'), SX.sym('qy'), SX.sym('qz')
    x = vertcat(px, py, pz, vx, vy, vz, qw, qx, qy, qz)
    q = vertcat(qw, qx, qy, qz)
    v = vertcat(vx, vy, vz)

    # Define controls
    T = SX.sym('T')
    wx, wy, wz = SX.sym('wx'), SX.sym('wy'), SX.sym('wz')
    u = vertcat(T, wx, wy, wz)
    omega = vertcat(wx, wy, wz)

    # Define parameters
    mass = SX.sym('m')
    kx, ky, kz, kh = SX.sym('kx'), SX.sym('ky'), SX.sym('kz'), SX.sym('kh')
    p_params = vertcat(mass, kx, ky, kz, kh)

    # Dynamics (EXACTLY matching test_dynamics.py including horizontal drag term)
    R = horzcat(
        vertcat(1 - 2*qy**2 - 2*qz**2, 2*qx*qy + 2*qz*qw, 2*qx*qz - 2*qy*qw),
        vertcat(2*qx*qy - 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz + 2*qx*qw),
        vertcat(2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx**2 - 2*qy**2)
    )

    g = vertcat(0, 0, -9.81)
    thrust_body = vertcat(0, 0, T)
    thrust_world = R @ thrust_body

    # Drag forces (INCLUDING horizontal drag contribution)
    v_body = R.T @ v
    vx_b, vy_b, vz_b = v_body[0], v_body[1], v_body[2]

    drag_force_body_x = -kx * vx_b
    drag_force_body_y = -ky * vy_b
    drag_force_body_z = -kz * vz_b + kh * (vx_b**2 + vy_b**2)  # CRITICAL: Include horizontal drag

    drag_force_body = vertcat(drag_force_body_x, drag_force_body_y, drag_force_body_z)
    drag_force_world = R @ drag_force_body

    # Velocity dynamics
    v_dot = g + (1 / mass) * (thrust_world + drag_force_world)

    # Quaternion dynamics
    omega_mat = vertcat(
        horzcat(0, -omega[0], -omega[1], -omega[2]),
        horzcat(omega[0], 0, omega[2], -omega[1]),
        horzcat(omega[1], -omega[2], 0, omega[0]),
        horzcat(omega[2], omega[1], -omega[0], 0)
    )
    q_dot = 0.5 * omega_mat @ q

    # Position dynamics
    p_dot = v

    # Combine into full state derivative
    x_dot_expr = vertcat(p_dot, v_dot, q_dot)

    # Create model
    model = AcadosModel()
    model.f_expl_expr = x_dot_expr
    model.x = x
    model.u = u
    model.p = p_params
    model.name = model_name
    model.con_h_expr = q.T @ q

    return model

def create_torque_model() -> AcadosModel:
    """Creates the quadrotor torque model"""
    model_name = 'quadrotor_torque'

    # Define states (13 states for torque model)
    px, py, pz = SX.sym('px'), SX.sym('py'), SX.sym('pz')
    vx, vy, vz = SX.sym('vx'), SX.sym('vy'), SX.sym('vz')
    qw, qx, qy, qz = SX.sym('qw'), SX.sym('qx'), SX.sym('qy'), SX.sym('qz')
    wx, wy, wz = SX.sym('wx'), SX.sym('wy'), SX.sym('wz')
    x = vertcat(px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz)
    v = vertcat(vx, vy, vz)
    q = vertcat(qw, qx, qy, qz)
    omega = vertcat(wx, wy, wz)

    # Define controls
    T = SX.sym('T')
    tau_x, tau_y, tau_z = SX.sym('tau_x'), SX.sym('tau_y'), SX.sym('tau_z')
    u = vertcat(T, tau_x, tau_y, tau_z)
    tau = vertcat(tau_x, tau_y, tau_z)

    # Define parameters
    mass = SX.sym('m')
    Ixx, Iyy, Izz = SX.sym('Ixx'), SX.sym('Iyy'), SX.sym('Izz')
    kx, ky, kz, kh = SX.sym('kx'), SX.sym('ky'), SX.sym('kz'), SX.sym('kh')
    p_params = vertcat(mass, Ixx, Iyy, Izz, kx, ky, kz, kh)
    I = vertcat(horzcat(Ixx, 0, 0), horzcat(0, Iyy, 0), horzcat(0, 0, Izz))

    # Use the same dynamics for position and velocity as the rate model
    R = horzcat(
        vertcat(1 - 2*qy**2 - 2*qz**2, 2*qx*qy + 2*qz*qw, 2*qx*qz - 2*qy*qw),
        vertcat(2*qx*qy - 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz + 2*qx*qw),
        vertcat(2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx**2 - 2*qy**2)
    )

    g = vertcat(0, 0, -9.81)
    thrust_body = vertcat(0, 0, T)
    thrust_world = R @ thrust_body

    # Drag forces (INCLUDING horizontal drag contribution)
    v_body = R.T @ v
    vx_b, vy_b, vz_b = v_body[0], v_body[1], v_body[2]

    drag_force_body_x = -kx * vx_b
    drag_force_body_y = -ky * vy_b
    drag_force_body_z = -kz * vz_b + kh * (vx_b**2 + vy_b**2)

    drag_force_body = vertcat(drag_force_body_x, drag_force_body_y, drag_force_body_z)
    drag_force_world = R @ drag_force_body

    # Dynamics
    p_dot = v
    v_dot = g + (1 / mass) * (thrust_world + drag_force_world)

    omega_mat = vertcat(
        horzcat(0, -omega[0], -omega[1], -omega[2]),
        horzcat(omega[0], 0, omega[2], -omega[1]),
        horzcat(omega[1], -omega[2], 0, omega[0]),
        horzcat(omega[2], omega[1], -omega[0], 0)
    )
    q_dot = 0.5 * omega_mat @ q

    # Angular acceleration
    omega_dot = inv(I) @ (tau - cross(omega, I @ omega))

    # Full state derivative
    x_dot_expr = vertcat(p_dot, v_dot, q_dot, omega_dot)

    # Create model
    model = AcadosModel()
    model.f_expl_expr = x_dot_expr
    model.x = x
    model.u = u
    model.p = p_params
    model.name = model_name
    model.con_h_expr = q.T @ q

    return model

def generate_solver(model: AcadosModel, N_horizon: int, Tf: float,
                    Q_diag: list, R_diag: list, lbu: list, ubu: list,
                    package_root: Path):
    """Generate solver with code export to package c_generated_code directory"""
    ocp = AcadosOcp()
    ocp.model = model

    # Get dimensions
    nx = model.x.shape[0]
    nu = model.u.shape[0]
    ny = nx + nu
    ny_e = nx

    # Set dimensions
    ocp.dims.N = N_horizon

    # Cost configuration (matching test_dynamics.py exactly)
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Constraints (matching test_dynamics.py exactly)
    ocp.constraints.lbu = np.array(lbu)
    ocp.constraints.ubu = np.array(ubu)
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.lh = np.array([0.99])
    ocp.constraints.uh = np.array([1.01])

    # Initial state constraint
    ocp.constraints.x0 = np.zeros(nx)

    # Parameters
    if model.p is not None:
        nparam = model.p.shape[0]
        if model.name == 'quadrotor_rate':
            # Default parameters matching test_dynamics.py
            ocp.parameter_values = np.array([1.47, 0.1, 0.1, 0.1, 0.0])  # mass, kx, ky, kz, kh
        else:  # torque model
            ocp.parameter_values = np.array([1.47, 0.03, 0.03, 0.05, 0.1, 0.1, 0.1, 0.0])

    # Solver options (EXACTLY matching test_dynamics.py)
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.tf = Tf
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.qp_solver_cond_N = int(N_horizon / 2)
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    # Improved tolerances for constraint satisfaction
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.qp_solver_tol_stat = 1e-6
    ocp.solver_options.qp_solver_tol_eq = 1e-6
    ocp.solver_options.qp_solver_tol_ineq = 1e-6
    ocp.solver_options.qp_solver_tol_comp = 1e-6

    # Allow more iterations for proper convergence
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.qp_solver_iter_max = 100

    # Set code export directory to package root/c_generated_code/model_name
    code_export_dir = package_root / 'c_generated_code' / model.name
    code_export_dir.mkdir(parents=True, exist_ok=True)

    ocp.code_export_directory = str(code_export_dir)

    # Create the solver (this generates the code)
    json_file = code_export_dir / f'{model.name}.json'
    solver = AcadosOcpSolver(ocp, json_file=str(json_file))

    return solver

if __name__ == '__main__':
    # Get package root directory
    package_root = get_package_root()
    print(f"Package root: {package_root}")

    # Ensure c_generated_code directory exists
    generated_code_dir = package_root / 'c_generated_code'
    generated_code_dir.mkdir(exist_ok=True)

    # Parameters EXACTLY matching test_dynamics.py
    N_horizon = 20
    Tf = 1.0  # Total time horizon

    try:
        # Generate RATE solver
        cleanup_generated_code('quadrotor_rate', package_root)
        print("Generating RATE controller solver...")

        # Cost matrices and constraints EXACTLY matching test_dynamics.py
        Q_rate = [100, 100, 100, 0, 0, 0, 0, 0, 0, 0]  # Position weighted, velocity and quaternion less
        R_rate = [0.1, 0.05, 0.05, 0.02]
        lbu_rate = [0.1, -4.0, -4.0, -2.0]
        ubu_rate = [20.0, 4.0, 4.0, 2.0]

        solver_rate = generate_solver(
            create_rate_model(),
            N_horizon,
            Tf,
            Q_rate,
            R_rate,
            lbu_rate,
            ubu_rate,
            package_root
        )
        print(f"-> RATE solver generated successfully in {package_root / 'c_generated_code' / 'quadrotor_rate'}")

        # Generate TORQUE solver
        cleanup_generated_code('quadrotor_torque', package_root)
        print("\nGenerating TORQUE controller solver...")

        # Cost matrices for torque model (13 states)
        Q_torque = [100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.01]
        R_torque = [0.1, 0.05, 0.05, 0.02]
        lbu_torque = [0.1, -0.5, -0.5, -0.1]
        ubu_torque = [20.0, 0.5, 0.5, 0.1]

        solver_torque = generate_solver(
            create_torque_model(),
            N_horizon,
            Tf,
            Q_torque,
            R_torque,
            lbu_torque,
            ubu_torque,
            package_root
        )
        print(f"-> TORQUE solver generated successfully in {package_root / 'c_generated_code' / 'quadrotor_torque'}")

        print("\n✅✅✅ All solvers generated successfully! ✅✅✅")
        print("\nGenerated code structure:")
        print(f"  {package_root}/")
        print(f"  └── c_generated_code/")
        print(f"      ├── quadrotor_rate/")
        print(f"      │   ├── acados_solver_quadrotor_rate.c")
        print(f"      │   ├── acados_solver_quadrotor_rate.h")
        print(f"      │   └── ...")
        print(f"      └── quadrotor_torque/")
        print(f"          ├── acados_solver_quadrotor_torque.c")
        print(f"          ├── acados_solver_quadrotor_torque.h")
        print(f"          └── ...")
        print("\nThe generated C code can now be compiled in CMakeLists.txt")

    except Exception as e:
        print(f"\n❌ An error occurred during solver generation: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
