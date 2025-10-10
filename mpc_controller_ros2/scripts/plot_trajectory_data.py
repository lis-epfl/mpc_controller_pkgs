#!/usr/bin/env python3
"""
Plot MPC trajectory tracking data from logged CSV files
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
from pathlib import Path

def load_data(log_directory):
    """Load state and horizon data from CSV files"""
    states_file = Path(log_directory) / "states.csv"
    horizon_file = Path(log_directory) / "mpc_horizon.csv"

    if not states_file.exists():
        raise FileNotFoundError(f"States file not found: {states_file}")

    states_df = pd.read_csv(states_file)

    horizon_df = None
    if horizon_file.exists():
        horizon_df = pd.read_csv(horizon_file)

    return states_df, horizon_df

def plot_position_tracking(states_df, save_path=None):
    """Plot position tracking: reference vs actual"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    time = (states_df['timestamp'] - states_df['timestamp'].iloc[0]).values

    # Position plots
    labels = ['X', 'Y', 'Z']
    actual_cols = ['x', 'y', 'z']
    ref_cols = ['ref_x', 'ref_y', 'ref_z']

    for i, (ax, label, actual, ref) in enumerate(zip(axes, labels, actual_cols, ref_cols)):
        ax.plot(time, states_df[actual].values, 'b-', label='Actual', linewidth=1.5)
        ax.plot(time, states_df[ref].values, 'r--', label='Reference', linewidth=1.5)
        ax.set_ylabel(f'{label} Position (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')

        # Calculate and display RMSE
        rmse = np.sqrt(np.mean((states_df[actual].values - states_df[ref].values)**2))
        ax.text(0.02, 0.95, f'RMSE: {rmse:.4f} m', transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Position Tracking Performance', fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_3d_trajectory(states_df, save_path=None):
    """Plot 3D trajectory comparison"""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Plot actual trajectory - Convert to numpy arrays
    ax.plot(states_df['x'].values, states_df['y'].values, states_df['z'].values,
            'b-', label='Actual', linewidth=2)

    # Plot reference trajectory - Convert to numpy arrays
    ax.plot(states_df['ref_x'].values, states_df['ref_y'].values, states_df['ref_z'].values,
            'r--', label='Reference', linewidth=2, alpha=0.7)

    # Mark start and end points
    ax.scatter(states_df['x'].iloc[0], states_df['y'].iloc[0], states_df['z'].iloc[0],
              color='green', s=100, label='Start', marker='o')
    ax.scatter(states_df['x'].iloc[-1], states_df['y'].iloc[-1], states_df['z'].iloc[-1],
              color='red', s=100, label='End', marker='*')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory Comparison', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Make axes equal
    max_range = np.array([
        states_df[['x', 'ref_x']].max().max() - states_df[['x', 'ref_x']].min().min(),
        states_df[['y', 'ref_y']].max().max() - states_df[['y', 'ref_y']].min().min(),
        states_df[['z', 'ref_z']].max().max() - states_df[['z', 'ref_z']].min().min()
    ]).max() / 2.0

    mid_x = (states_df[['x', 'ref_x']].max().max() + states_df[['x', 'ref_x']].min().min()) * 0.5
    mid_y = (states_df[['y', 'ref_y']].max().max() + states_df[['y', 'ref_y']].min().min()) * 0.5
    mid_z = (states_df[['z', 'ref_z']].max().max() + states_df[['z', 'ref_z']].min().min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_tracking_errors(states_df, save_path=None):
    """Plot tracking errors over time"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    time = (states_df['timestamp'] - states_df['timestamp'].iloc[0]).values

    # Component-wise errors
    ax = axes[0]
    ax.plot(time, states_df['error_x'].values, 'r-', label='X Error', alpha=0.7)
    ax.plot(time, states_df['error_y'].values, 'g-', label='Y Error', alpha=0.7)
    ax.plot(time, states_df['error_z'].values, 'b-', label='Z Error', alpha=0.7)
    ax.set_ylabel('Position Error (m)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Component-wise Tracking Errors')

    # Total error norm
    ax = axes[1]
    ax.plot(time, states_df['error_norm'].values, 'k-', linewidth=2)
    ax.set_ylabel('Total Error (m)')
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    ax.set_title('Total Tracking Error (Euclidean Norm)')

    # Add statistics
    mean_error = states_df['error_norm'].mean()
    max_error = states_df['error_norm'].max()
    std_error = states_df['error_norm'].std()

    stats_text = f'Mean: {mean_error:.4f} m\nMax: {max_error:.4f} m\nStd: {std_error:.4f} m'
    ax.text(0.98, 0.95, stats_text, transform=ax.transAxes,
            verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    fig.suptitle('Tracking Error Analysis', fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_control_inputs(states_df, save_path=None):
    """Plot control inputs over time"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True)

    time = (states_df['timestamp'] - states_df['timestamp'].iloc[0]).values

    # Detect if this is torque model (has non-zero angular velocities) or rate model
    is_torque_model = ('wx' in states_df.columns) and (
        states_df['wx'].abs().max() > 0 or
        states_df['wy'].abs().max() > 0 or
        states_df['wz'].abs().max() > 0)

    # Set appropriate labels based on model type
    if is_torque_model:
        control_labels = ['Thrust (N)', 'Roll Torque τx (Nm)', 'Pitch Torque τy (Nm)', 'Yaw Torque τz (Nm)']
        model_type = "Torque Model"
    else:
        control_labels = ['Thrust (N)', 'Roll Rate ωx (rad/s)', 'Pitch Rate ωy (rad/s)', 'Yaw Rate ωz (rad/s)']
        model_type = "Rate Model"

    control_cols = ['thrust', 'u1', 'u2', 'u3']

    for ax, label, col in zip(axes, control_labels, control_cols):
        ax.plot(time, states_df[col].values, 'b-', linewidth=1.5)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

        # Add mean line
        mean_val = states_df[col].mean()
        ax.axhline(y=mean_val, color='r', linestyle='--', alpha=0.5,
                  label=f'Mean: {mean_val:.3f}')
        ax.legend(loc='upper right')

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle(f'Control Inputs ({model_type})', fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_velocity_tracking(states_df, save_path=None):
    """Plot velocity tracking"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    time = (states_df['timestamp'] - states_df['timestamp'].iloc[0]).values

    # Velocity plots
    labels = ['Vx', 'Vy', 'Vz']
    actual_cols = ['vx', 'vy', 'vz']
    ref_cols = ['ref_vx', 'ref_vy', 'ref_vz']

    for i, (ax, label, actual, ref) in enumerate(zip(axes, labels, actual_cols, ref_cols)):
        ax.plot(time, states_df[actual].values, 'b-', label='Actual', linewidth=1.5)
        ax.plot(time, states_df[ref].values, 'r--', label='Reference', linewidth=1.5)
        ax.set_ylabel(f'{label} (m/s)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')

        # Calculate RMSE
        rmse = np.sqrt(np.mean((states_df[actual].values - states_df[ref].values)**2))
        ax.text(0.02, 0.95, f'RMSE: {rmse:.4f} m/s', transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Velocity Tracking Performance', fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def estimate_inertia_matrix(states_df, max_tau_c=0.1, dt=0.01):
    """
    Estimate the inertia matrix and actuator dynamics from logged torque commands and angular velocities.
    Models actuator dynamics as a first-order system: τ_actual = τ_cmd / (τs + 1)

    Args:
        states_df: DataFrame with logged states and commands
        max_tau_c: Maximum time constant to consider in seconds (default: 0.1s = 100ms)
        dt: Sample time in seconds (derived from timestamp differences)

    Returns:
        dict with estimated parameters and diagnostics
    """
    import scipy.signal
    from scipy.optimize import least_squares, differential_evolution

    # Check if we have the necessary data
    if 'wx' not in states_df.columns or states_df['wx'].abs().max() == 0:
        print("No angular velocity data available for inertia estimation")
        return None

    print("\n=== Inertia Matrix and Actuator Dynamics Estimation ===")

    # Extract time series
    time = states_df['timestamp'].values
    if len(time) < 50:
        print("Insufficient data points for estimation (need at least 50)")
        return None

    # Calculate actual dt from timestamps
    dt_actual = np.median(np.diff(time))
    print(f"Sample time: {dt_actual:.4f} s ({1/dt_actual:.1f} Hz)")

    # Extract angular velocities and torque commands
    omega_x = states_df['wx'].values
    omega_y = states_df['wy'].values
    omega_z = states_df['wz'].values

    tau_cmd_x = states_df['u1'].values  # Roll torque command
    tau_cmd_y = states_df['u2'].values  # Pitch torque command
    tau_cmd_z = states_df['u3'].values  # Yaw torque command

    # Filter the signals to reduce noise (using Savitzky-Golay filter)
    from scipy.signal import savgol_filter
    window_length = min(21, len(omega_x) // 4)
    if window_length % 2 == 0:
        window_length -= 1
    window_length = max(5, window_length)

    omega_x_filt = savgol_filter(omega_x, window_length, 3)
    omega_y_filt = savgol_filter(omega_y, window_length, 3)
    omega_z_filt = savgol_filter(omega_z, window_length, 3)

    # Calculate angular accelerations using finite differences
    omega_dot_x = np.gradient(omega_x_filt, dt_actual)
    omega_dot_y = np.gradient(omega_y_filt, dt_actual)
    omega_dot_z = np.gradient(omega_z_filt, dt_actual)

    # Apply additional filtering to accelerations
    omega_dot_x = savgol_filter(omega_dot_x, window_length, 3)
    omega_dot_y = savgol_filter(omega_dot_y, window_length, 3)
    omega_dot_z = savgol_filter(omega_dot_z, window_length, 3)

    def simulate_first_order_actuator(tau_cmd, tau_c, dt):
        """
        Simulate first-order actuator dynamics: tau_dot = (tau_cmd - tau_actual) / tau_c
        Using discrete-time approximation: tau[k+1] = alpha * tau_cmd[k] + (1-alpha) * tau[k]
        where alpha = dt / (tau_c + dt)
        """
        alpha = dt / (tau_c + dt)
        tau_actual = np.zeros_like(tau_cmd)
        tau_actual[0] = tau_cmd[0] * alpha  # Initial condition

        for k in range(1, len(tau_cmd)):
            tau_actual[k] = alpha * tau_cmd[k] + (1 - alpha) * tau_actual[k-1]

        return tau_actual

    # Skip initial transient and final samples
    skip_start = 10
    skip_end = 10

    # Grid search for initial time constant estimate
    print("\nSearching for optimal actuator time constant...")
    tau_c_values = np.linspace(0.001, max_tau_c, 20)
    best_tau_c = 0.01
    best_rmse = float('inf')
    best_initial_params = None

    for tau_c_test in tau_c_values:
        # Simulate actuator dynamics with this time constant
        tau_actual_x = simulate_first_order_actuator(tau_cmd_x, tau_c_test, dt_actual)
        tau_actual_y = simulate_first_order_actuator(tau_cmd_y, tau_c_test, dt_actual)
        tau_actual_z = simulate_first_order_actuator(tau_cmd_z, tau_c_test, dt_actual)

        # Simple estimation ignoring gyroscopic effects
        valid_x = np.abs(omega_dot_x[skip_start:-skip_end]) > 0.01
        valid_y = np.abs(omega_dot_y[skip_start:-skip_end]) > 0.01
        valid_z = np.abs(omega_dot_z[skip_start:-skip_end]) > 0.01

        if np.sum(valid_x) > 10 and np.sum(valid_y) > 10 and np.sum(valid_z) > 10:
            # Estimate inertias
            Ixx_est = np.median(tau_actual_x[skip_start:-skip_end][valid_x] /
                               omega_dot_x[skip_start:-skip_end][valid_x])
            Iyy_est = np.median(tau_actual_y[skip_start:-skip_end][valid_y] /
                               omega_dot_y[skip_start:-skip_end][valid_y])
            Izz_est = np.median(tau_actual_z[skip_start:-skip_end][valid_z] /
                               omega_dot_z[skip_start:-skip_end][valid_z])

            # Ensure positive values
            Ixx_est = max(abs(Ixx_est), 0.001)
            Iyy_est = max(abs(Iyy_est), 0.001)
            Izz_est = max(abs(Izz_est), 0.001)

            # Calculate prediction error
            pred_omega_dot_x = tau_actual_x[skip_start:-skip_end] / Ixx_est
            pred_omega_dot_y = tau_actual_y[skip_start:-skip_end] / Iyy_est
            pred_omega_dot_z = tau_actual_z[skip_start:-skip_end] / Izz_est

            rmse = np.sqrt(np.mean((pred_omega_dot_x - omega_dot_x[skip_start:-skip_end])**2 +
                                   (pred_omega_dot_y - omega_dot_y[skip_start:-skip_end])**2 +
                                   (pred_omega_dot_z - omega_dot_z[skip_start:-skip_end])**2))

            if rmse < best_rmse:
                best_rmse = rmse
                best_tau_c = tau_c_test
                best_initial_params = [Ixx_est, Iyy_est, Izz_est]

    if best_initial_params is None:
        print("Failed to find initial estimate - insufficient angular acceleration data")
        return None

    print(f"Initial time constant estimate: τ_c = {best_tau_c:.4f} s ({best_tau_c*1000:.1f} ms)")
    print(f"Initial inertia estimates: Ixx={best_initial_params[0]:.6f}, Iyy={best_initial_params[1]:.6f}, Izz={best_initial_params[2]:.6f}")

    # Joint optimization of all parameters including gyroscopic effects
    print("\nRefining estimates with full dynamics and actuator model...")

    def residuals(params):
        Ixx, Iyy, Izz, tau_c = params

        # Ensure positive values
        Ixx = max(abs(Ixx), 0.0001)
        Iyy = max(abs(Iyy), 0.0001)
        Izz = max(abs(Izz), 0.0001)
        tau_c = max(abs(tau_c), 0.0001)

        # Simulate actuator dynamics
        tau_actual_x = simulate_first_order_actuator(tau_cmd_x, tau_c, dt_actual)
        tau_actual_y = simulate_first_order_actuator(tau_cmd_y, tau_c, dt_actual)
        tau_actual_z = simulate_first_order_actuator(tau_cmd_z, tau_c, dt_actual)

        # Calculate predicted angular accelerations with gyroscopic effects
        pred_omega_dot_x = (tau_actual_x[skip_start:-skip_end] -
                            omega_y_filt[skip_start:-skip_end] * omega_z_filt[skip_start:-skip_end] * (Izz - Iyy)) / Ixx
        pred_omega_dot_y = (tau_actual_y[skip_start:-skip_end] -
                            omega_z_filt[skip_start:-skip_end] * omega_x_filt[skip_start:-skip_end] * (Ixx - Izz)) / Iyy
        pred_omega_dot_z = (tau_actual_z[skip_start:-skip_end] -
                            omega_x_filt[skip_start:-skip_end] * omega_y_filt[skip_start:-skip_end] * (Iyy - Ixx)) / Izz

        # Weight errors by magnitude of acceleration (focus on dynamic regions)
        weight_x = np.abs(omega_dot_x[skip_start:-skip_end]) + 0.1
        weight_y = np.abs(omega_dot_y[skip_start:-skip_end]) + 0.1
        weight_z = np.abs(omega_dot_z[skip_start:-skip_end]) + 0.1

        res = np.concatenate([
            (pred_omega_dot_x - omega_dot_x[skip_start:-skip_end]) * weight_x,
            (pred_omega_dot_y - omega_dot_y[skip_start:-skip_end]) * weight_y,
            (pred_omega_dot_z - omega_dot_z[skip_start:-skip_end]) * weight_z
        ])

        return res

    # Initial guess: [Ixx, Iyy, Izz, tau_c]
    initial_guess = best_initial_params + [best_tau_c]

    # Bounds for optimization
    bounds = ([0.0001, 0.0001, 0.0001, 0.0001],  # Lower bounds
              [1.0, 1.0, 1.0, max_tau_c])         # Upper bounds

    # Run optimization
    result = least_squares(residuals, initial_guess, bounds=bounds, max_nfev=1000)

    Ixx_final, Iyy_final, Izz_final, tau_c_final = result.x

    # Calculate final actual torques for visualization
    tau_actual_x_final = simulate_first_order_actuator(tau_cmd_x, tau_c_final, dt_actual)
    tau_actual_y_final = simulate_first_order_actuator(tau_cmd_y, tau_c_final, dt_actual)
    tau_actual_z_final = simulate_first_order_actuator(tau_cmd_z, tau_c_final, dt_actual)

    # Calculate final predictions
    pred_omega_dot_x_final = (tau_actual_x_final -
                              omega_y_filt * omega_z_filt * (Izz_final - Iyy_final)) / Ixx_final
    pred_omega_dot_y_final = (tau_actual_y_final -
                              omega_z_filt * omega_x_filt * (Ixx_final - Izz_final)) / Iyy_final
    pred_omega_dot_z_final = (tau_actual_z_final -
                              omega_x_filt * omega_y_filt * (Iyy_final - Ixx_final)) / Izz_final

    # Calculate R-squared values
    ss_res_x = np.sum((omega_dot_x[skip_start:-skip_end] - pred_omega_dot_x_final[skip_start:-skip_end])**2)
    ss_tot_x = np.sum((omega_dot_x[skip_start:-skip_end] - np.mean(omega_dot_x[skip_start:-skip_end]))**2)
    r2_x = 1 - (ss_res_x / ss_tot_x) if ss_tot_x > 0 else 0

    ss_res_y = np.sum((omega_dot_y[skip_start:-skip_end] - pred_omega_dot_y_final[skip_start:-skip_end])**2)
    ss_tot_y = np.sum((omega_dot_y[skip_start:-skip_end] - np.mean(omega_dot_y[skip_start:-skip_end]))**2)
    r2_y = 1 - (ss_res_y / ss_tot_y) if ss_tot_y > 0 else 0

    ss_res_z = np.sum((omega_dot_z[skip_start:-skip_end] - pred_omega_dot_z_final[skip_start:-skip_end])**2)
    ss_tot_z = np.sum((omega_dot_z[skip_start:-skip_end] - np.mean(omega_dot_z[skip_start:-skip_end]))**2)
    r2_z = 1 - (ss_res_z / ss_tot_z) if ss_tot_z > 0 else 0

    # Calculate equivalent delay from time constant (63% rise time approximation)
    equivalent_delay_ms = tau_c_final * 1000

    print("\n=== Estimation Results ===")
    print("\nInertia Matrix:")
    print(f"  Ixx = {Ixx_final:.6f} kg⋅m²")
    print(f"  Iyy = {Iyy_final:.6f} kg⋅m²")
    print(f"  Izz = {Izz_final:.6f} kg⋅m²")
    print("\nActuator Dynamics:")
    print(f"  Time constant τ_c = {tau_c_final:.4f} s ({tau_c_final*1000:.1f} ms)")
    print(f"  Bandwidth ≈ {1/(2*np.pi*tau_c_final):.1f} Hz")
    print(f"  Equivalent delay ≈ {equivalent_delay_ms:.1f} ms")
    print("\nGoodness of Fit (R²):")
    print(f"  Roll (X):  {r2_x:.3f}")
    print(f"  Pitch (Y): {r2_y:.3f}")
    print(f"  Yaw (Z):   {r2_z:.3f}")

    # Create validation plots
    fig, axes = plt.subplots(3, 3, figsize=(18, 12))
    time_plot = time[skip_start:-skip_end] - time[0]

    # Plot for each axis
    for i, (omega, omega_dot, omega_dot_pred, tau_cmd, tau_actual, axis_name, I_est) in enumerate([
        (omega_x_filt, omega_dot_x, pred_omega_dot_x_final, tau_cmd_x, tau_actual_x_final, 'Roll (X)', Ixx_final),
        (omega_y_filt, omega_dot_y, pred_omega_dot_y_final, tau_cmd_y, tau_actual_y_final, 'Pitch (Y)', Iyy_final),
        (omega_z_filt, omega_dot_z, pred_omega_dot_z_final, tau_cmd_z, tau_actual_z_final, 'Yaw (Z)', Izz_final)
    ]):
        # Left column: Angular acceleration comparison
        ax = axes[i, 0]
        ax.plot(time_plot, omega_dot[skip_start:-skip_end], 'b-', label='Measured', alpha=0.7, linewidth=1.5)
        ax.plot(time_plot, omega_dot_pred[skip_start:-skip_end], 'r--', label='Predicted', linewidth=1.5)
        ax.set_ylabel(f'{axis_name} ω̇ (rad/s²)')
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{axis_name} Angular Acceleration (I={I_est:.6f} kg⋅m²)')

        # Middle column: Torque command vs actual torque
        ax = axes[i, 1]
        ax.plot(time_plot, tau_cmd[skip_start:-skip_end], 'g-', label='Command', alpha=0.7, linewidth=1.5)
        ax.plot(time_plot, tau_actual[skip_start:-skip_end], 'b--', label='Actual (est.)', linewidth=1.5)
        ax.set_ylabel('Torque (Nm)')
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{axis_name} Torque: Command vs Actual')

        # Right column: Angular velocity and actual torque
        ax = axes[i, 2]
        ax.plot(time_plot, tau_actual[skip_start:-skip_end], 'g-', label='τ_actual (Nm)', linewidth=1.5)
        ax.set_ylabel('Actual Torque (Nm)', color='g')
        ax.tick_params(axis='y', labelcolor='g')
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(time_plot, omega[skip_start:-skip_end], 'b-', alpha=0.7, label='ω (rad/s)', linewidth=1.5)
        ax2.set_ylabel('Angular Velocity (rad/s)', color='b')
        ax2.tick_params(axis='y', labelcolor='b')

        ax.set_title(f'{axis_name} Actual Torque vs Angular Velocity')

    fig.suptitle(f'Inertia and Actuator Dynamics Estimation\nτ_c = {tau_c_final*1000:.1f} ms, ' +
                f'Bandwidth ≈ {1/(2*np.pi*tau_c_final):.1f} Hz',
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()

    # Plot actuator step response
    fig2, ax = plt.subplots(1, 1, figsize=(8, 6))

    # Generate step response
    t_step = np.arange(0, 10*tau_c_final, dt_actual)
    step_input = np.ones_like(t_step)
    step_response = simulate_first_order_actuator(step_input, tau_c_final, dt_actual)

    ax.plot(t_step*1000, step_input, 'g--', label='Command', linewidth=2)
    ax.plot(t_step*1000, step_response, 'b-', label='Response', linewidth=2)
    ax.axhline(y=0.632, color='r', linestyle=':', alpha=0.5, label=f'63.2% at τ_c={tau_c_final*1000:.1f}ms')
    ax.axvline(x=tau_c_final*1000, color='r', linestyle=':', alpha=0.5)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Normalized Torque')
    ax.set_title(f'Estimated Actuator Step Response\nFirst-order: τ/(τ_c·s + 1), τ_c = {tau_c_final*1000:.1f} ms')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, min(5*tau_c_final*1000, t_step[-1]*1000)])
    plt.tight_layout()
    plt.show()

    return {
        'Ixx': Ixx_final,
        'Iyy': Iyy_final,
        'Izz': Izz_final,
        'tau_c': tau_c_final,
        'tau_c_ms': tau_c_final * 1000,
        'bandwidth_hz': 1/(2*np.pi*tau_c_final),
        'r2_x': r2_x,
        'r2_y': r2_y,
        'r2_z': r2_z,
        'sample_rate_hz': 1/dt_actual
    }

def plot_all_combined(states_df, horizon_df=None, save_path=None):
    """Plot all states and controls with MPC predictions overlaid"""

    # Check if angular velocities are available (torque model)
    has_angular_velocity = ('wx' in states_df.columns) and (states_df['wx'].abs().max() > 0 or
                           states_df['wy'].abs().max() > 0 or
                           states_df['wz'].abs().max() > 0)

    if has_angular_velocity:
        # Create 4x3 grid for torque model (includes angular velocities)
        fig, axes = plt.subplots(4, 3, figsize=(18, 16), sharex=True)
    else:
        # Original 3x3 grid for rate model
        fig, axes = plt.subplots(3, 3, figsize=(18, 12), sharex=True)

    time = (states_df['timestamp'] - states_df['timestamp'].iloc[0]).values

    # Prepare MPC horizon data if available
    horizon_lines_to_plot = []
    if horizon_df is not None:
        unique_timestamps = horizon_df['timestamp'].unique()
        # Sample every 5th timestamp (approximately every 0.25 seconds at 50Hz)
        step_size = 5
        sampled_timestamps = unique_timestamps[::step_size]

        # Limit number of horizons shown for clarity
        max_horizons = 20
        if len(sampled_timestamps) > max_horizons:
            indices = np.linspace(0, len(sampled_timestamps)-1, max_horizons, dtype=int)
            sampled_timestamps = sampled_timestamps[indices]

        for ts in sampled_timestamps:
            horizon_data = horizon_df[horizon_df['timestamp'] == ts]
            if not horizon_data.empty:
                # Create time array for this horizon prediction
                dt = 0.05  # 50Hz MPC rate
                horizon_time = ts - states_df['timestamp'].iloc[0] + horizon_data['horizon_idx'].values * dt
                horizon_lines_to_plot.append({
                    'time': horizon_time,
                    'data': horizon_data,
                    'start_time': ts - states_df['timestamp'].iloc[0]
                })

    # Column 1: X, Vx, (Wx if torque), u2
    # X position
    ax = axes[0, 0]
    ax.plot(time, states_df['x'].values, 'b-', label='Actual', linewidth=2, zorder=10)
    ax.plot(time, states_df['ref_x'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

    # Add MPC predictions for X
    for i, horizon in enumerate(horizon_lines_to_plot):
        ax.plot(horizon['time'], horizon['data']['x'].values, 'c-',
                alpha=0.7, linewidth=0.8, zorder=5)
        if i == 0:  # Add label only once
            ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

    ax.set_ylabel('X Position (m)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    rmse = np.sqrt(np.mean((states_df['x'].values - states_df['ref_x'].values)**2))
    ax.set_title(f'X Position (RMSE: {rmse:.3f} m)', fontsize=10)

    # Vx velocity
    ax = axes[1, 0]
    ax.plot(time, states_df['vx'].values, 'b-', label='Actual', linewidth=2, zorder=10)
    ax.plot(time, states_df['ref_vx'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

    # Add MPC predictions for Vx
    for i, horizon in enumerate(horizon_lines_to_plot):
        ax.plot(horizon['time'], horizon['data']['vx'].values, 'c-',
                alpha=0.7, linewidth=0.8, zorder=5)
        if i == 0:
            ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

    ax.set_ylabel('Vx (m/s)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    rmse = np.sqrt(np.mean((states_df['vx'].values - states_df['ref_vx'].values)**2))
    ax.set_title(f'X Velocity (RMSE: {rmse:.3f} m/s)', fontsize=10)

    if has_angular_velocity:
        # Wx angular velocity (for torque model)
        ax = axes[2, 0]
        ax.plot(time, states_df['wx'].values, 'b-', label='Actual', linewidth=2, zorder=10)
        ax.plot(time, states_df['ref_wx'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

        # Add MPC predictions for Wx
        for i, horizon in enumerate(horizon_lines_to_plot):
            if 'wx' in horizon['data'].columns:
                ax.plot(horizon['time'], horizon['data']['wx'].values, 'c-',
                        alpha=0.7, linewidth=0.8, zorder=5)
                if i == 0:
                    ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

        ax.set_ylabel('ωx (rad/s)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        rmse = np.sqrt(np.mean((states_df['wx'].values - states_df['ref_wx'].values)**2))
        ax.set_title(f'Roll Rate (RMSE: {rmse:.3f} rad/s)', fontsize=10)

        # u2 (Pitch torque for torque model)
        ax = axes[3, 0]
        control_label = 'Pitch Torque (Nm)'
    else:
        # u2 (Pitch rate for rate model)
        ax = axes[2, 0]
        control_label = 'Pitch Rate (rad/s)'

    ax.plot(time, states_df['u2'].values, 'g-', linewidth=1.5)
    ax.set_ylabel(control_label)
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    mean_val = states_df['u2'].mean()
    std_val = states_df['u2'].std()
    ax.axhline(y=mean_val, color='r', linestyle='--', alpha=0.5)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_title(f'u2 (μ={mean_val:.3f}, σ={std_val:.3f})', fontsize=10)

    # Column 2: Y, Vy, (Wy if torque), u1
    # Y position
    ax = axes[0, 1]
    ax.plot(time, states_df['y'].values, 'b-', label='Actual', linewidth=2, zorder=10)
    ax.plot(time, states_df['ref_y'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

    # Add MPC predictions for Y
    for i, horizon in enumerate(horizon_lines_to_plot):
        ax.plot(horizon['time'], horizon['data']['y'].values, 'c-',
                alpha=0.7, linewidth=0.8, zorder=5)
        if i == 0:
            ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

    ax.set_ylabel('Y Position (m)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    rmse = np.sqrt(np.mean((states_df['y'].values - states_df['ref_y'].values)**2))
    ax.set_title(f'Y Position (RMSE: {rmse:.3f} m)', fontsize=10)

    # Vy velocity
    ax = axes[1, 1]
    ax.plot(time, states_df['vy'].values, 'b-', label='Actual', linewidth=2, zorder=10)
    ax.plot(time, states_df['ref_vy'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

    # Add MPC predictions for Vy
    for i, horizon in enumerate(horizon_lines_to_plot):
        ax.plot(horizon['time'], horizon['data']['vy'].values, 'c-',
                alpha=0.7, linewidth=0.8, zorder=5)
        if i == 0:
            ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

    ax.set_ylabel('Vy (m/s)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    rmse = np.sqrt(np.mean((states_df['vy'].values - states_df['ref_vy'].values)**2))
    ax.set_title(f'Y Velocity (RMSE: {rmse:.3f} m/s)', fontsize=10)

    if has_angular_velocity:
        # Wy angular velocity (for torque model)
        ax = axes[2, 1]
        ax.plot(time, states_df['wy'].values, 'b-', label='Actual', linewidth=2, zorder=10)
        ax.plot(time, states_df['ref_wy'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

        # Add MPC predictions for Wy
        for i, horizon in enumerate(horizon_lines_to_plot):
            if 'wy' in horizon['data'].columns:
                ax.plot(horizon['time'], horizon['data']['wy'].values, 'c-',
                        alpha=0.7, linewidth=0.8, zorder=5)
                if i == 0:
                    ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

        ax.set_ylabel('ωy (rad/s)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        rmse = np.sqrt(np.mean((states_df['wy'].values - states_df['ref_wy'].values)**2))
        ax.set_title(f'Pitch Rate (RMSE: {rmse:.3f} rad/s)', fontsize=10)

        # u1 (Roll torque for torque model)
        ax = axes[3, 1]
        control_label = 'Roll Torque (Nm)'
    else:
        # u1 (Roll rate for rate model)
        ax = axes[2, 1]
        control_label = 'Roll Rate (rad/s)'

    ax.plot(time, states_df['u1'].values, 'g-', linewidth=1.5)
    ax.set_ylabel(control_label)
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    mean_val = states_df['u1'].mean()
    std_val = states_df['u1'].std()
    ax.axhline(y=mean_val, color='r', linestyle='--', alpha=0.5)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_title(f'u1 (μ={mean_val:.3f}, σ={std_val:.3f})', fontsize=10)

    # Column 3: Z, Vz, (Wz if torque), Thrust
    # Z position
    ax = axes[0, 2]
    ax.plot(time, states_df['z'].values, 'b-', label='Actual', linewidth=2, zorder=10)
    ax.plot(time, states_df['ref_z'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

    # Add MPC predictions for Z
    for i, horizon in enumerate(horizon_lines_to_plot):
        ax.plot(horizon['time'], horizon['data']['z'].values, 'c-',
                alpha=0.7, linewidth=0.8, zorder=5)
        if i == 0:
            ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

    ax.set_ylabel('Z Position (m)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    rmse = np.sqrt(np.mean((states_df['z'].values - states_df['ref_z'].values)**2))
    ax.set_title(f'Z Position (RMSE: {rmse:.3f} m)', fontsize=10)

    # Vz velocity
    ax = axes[1, 2]
    ax.plot(time, states_df['vz'].values, 'b-', label='Actual', linewidth=2, zorder=10)
    ax.plot(time, states_df['ref_vz'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

    # Add MPC predictions for Vz
    for i, horizon in enumerate(horizon_lines_to_plot):
        ax.plot(horizon['time'], horizon['data']['vz'].values, 'c-',
                alpha=0.7, linewidth=0.8, zorder=5)
        if i == 0:
            ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

    ax.set_ylabel('Vz (m/s)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    rmse = np.sqrt(np.mean((states_df['vz'].values - states_df['ref_vz'].values)**2))
    ax.set_title(f'Z Velocity (RMSE: {rmse:.3f} m/s)', fontsize=10)

    if has_angular_velocity:
        # Wz angular velocity (for torque model)
        ax = axes[2, 2]
        ax.plot(time, states_df['wz'].values, 'b-', label='Actual', linewidth=2, zorder=10)
        ax.plot(time, states_df['ref_wz'].values, 'r--', label='Reference', linewidth=1.5, zorder=9)

        # Add MPC predictions for Wz
        for i, horizon in enumerate(horizon_lines_to_plot):
            if 'wz' in horizon['data'].columns:
                ax.plot(horizon['time'], horizon['data']['wz'].values, 'c-',
                        alpha=0.7, linewidth=0.8, zorder=5)
                if i == 0:
                    ax.plot([], [], 'c-', alpha=0.7, linewidth=1.5, label='MPC Predictions')

        ax.set_ylabel('ωz (rad/s)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        rmse = np.sqrt(np.mean((states_df['wz'].values - states_df['ref_wz'].values)**2))
        ax.set_title(f'Yaw Rate (RMSE: {rmse:.3f} rad/s)', fontsize=10)

        # Thrust
        ax = axes[3, 2]
    else:
        # Thrust
        ax = axes[2, 2]

    ax.plot(time, states_df['thrust'].values, 'g-', linewidth=1.5)
    ax.set_ylabel('Thrust (N)')
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    mean_val = states_df['thrust'].mean()
    std_val = states_df['thrust'].std()
    ax.axhline(y=mean_val, color='r', linestyle='--', alpha=0.5)
    ax.set_title(f'Thrust (μ={mean_val:.3f} N, σ={std_val:.3f})', fontsize=10)

    # Add info about MPC predictions and controller type
    controller_type = "Torque Model" if has_angular_velocity else "Rate Model"
    if horizon_df is not None and len(horizon_lines_to_plot) > 0:
        fig.suptitle(f'State and Control Tracking with MPC Predictions ({controller_type})\n'
                    f'({len(horizon_lines_to_plot)} horizon predictions shown)',
                    fontsize=14, fontweight='bold')
    else:
        fig.suptitle(f'State and Control Tracking ({controller_type})', fontsize=14, fontweight='bold')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_mpc_horizon_comparison(horizon_df, timestamp_to_plot=None, save_path=None):
    """Plot MPC prediction horizon with reference trajectory comparison"""
    if horizon_df is None:
        print("No horizon data available")
        return

    # If no timestamp specified, use the middle one
    if timestamp_to_plot is None:
        unique_timestamps = horizon_df['timestamp'].unique()
        timestamp_to_plot = unique_timestamps[len(unique_timestamps)//2]

    # Filter data for the specific timestamp
    horizon_data = horizon_df[horizon_df['timestamp'] == timestamp_to_plot]

    if horizon_data.empty:
        print(f"No data found for timestamp {timestamp_to_plot}")
        return

    # Detect if this is torque model or rate model
    is_torque_model = False
    if 'wx' in horizon_data.columns:
        is_torque_model = (horizon_data['wx'].abs().max() > 0 or
                          horizon_data['wy'].abs().max() > 0 or
                          horizon_data['wz'].abs().max() > 0)

    fig = plt.figure(figsize=(16, 12))

    # 3D trajectory comparison
    ax1 = fig.add_subplot(221, projection='3d')

    # Plot reference trajectory - Convert to numpy arrays
    ax1.plot(horizon_data['ref_x'].values, horizon_data['ref_y'].values, horizon_data['ref_z'].values,
            'r--', label='Reference', linewidth=2, markersize=6, marker='o', alpha=0.7)

    # Plot MPC predicted trajectory - Convert to numpy arrays
    ax1.plot(horizon_data['x'].values, horizon_data['y'].values, horizon_data['z'].values,
            'b.-', label='MPC Predicted', linewidth=2, markersize=8)

    # Highlight the starting point
    ax1.scatter(horizon_data['x'].iloc[0], horizon_data['y'].iloc[0], horizon_data['z'].iloc[0],
               color='green', s=150, marker='*', label='Current Position')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Reference vs MPC Prediction (3D)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Position comparison over horizon
    ax2 = fig.add_subplot(222)
    horizon_steps = horizon_data['horizon_idx'].values

    # Plot predicted positions
    ax2.plot(horizon_steps, horizon_data['x'].values, 'b-', label='Predicted X', linewidth=2)
    ax2.plot(horizon_steps, horizon_data['y'].values, 'g-', label='Predicted Y', linewidth=2)
    ax2.plot(horizon_steps, horizon_data['z'].values, 'm-', label='Predicted Z', linewidth=2)

    # Plot reference positions
    ax2.plot(horizon_steps, horizon_data['ref_x'].values, 'b--', label='Ref X', alpha=0.7)
    ax2.plot(horizon_steps, horizon_data['ref_y'].values, 'g--', label='Ref Y', alpha=0.7)
    ax2.plot(horizon_steps, horizon_data['ref_z'].values, 'm--', label='Ref Z', alpha=0.7)

    ax2.set_xlabel('Horizon Step')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position: Reference vs Predicted')
    ax2.legend(ncol=2, fontsize=8)
    ax2.grid(True, alpha=0.3)

    # Position tracking errors over horizon
    ax3 = fig.add_subplot(223)

    # Calculate errors
    error_x = horizon_data['x'].values - horizon_data['ref_x'].values
    error_y = horizon_data['y'].values - horizon_data['ref_y'].values
    error_z = horizon_data['z'].values - horizon_data['ref_z'].values
    error_norm = np.sqrt(error_x**2 + error_y**2 + error_z**2)

    ax3.plot(horizon_steps, error_x, 'r.-', label='X Error')
    ax3.plot(horizon_steps, error_y, 'g.-', label='Y Error')
    ax3.plot(horizon_steps, error_z, 'b.-', label='Z Error')
    ax3.plot(horizon_steps, error_norm, 'k.-', label='Total Error', linewidth=2)

    ax3.set_xlabel('Horizon Step')
    ax3.set_ylabel('Prediction Error (m)')
    ax3.set_title('MPC Prediction Error vs Reference')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Add RMS error text
    rms_error = np.sqrt(np.mean(error_norm**2))
    ax3.text(0.98, 0.95, f'RMS Error: {rms_error:.4f} m',
            transform=ax3.transAxes, ha='right', va='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Velocity and Control comparison
    ax4 = fig.add_subplot(224)

    # Calculate velocity magnitudes
    pred_vel_mag = np.sqrt(horizon_data['vx'].values**2 + horizon_data['vy'].values**2 + horizon_data['vz'].values**2)
    ref_vel_mag = np.sqrt(horizon_data['ref_vx'].values**2 + horizon_data['ref_vy'].values**2 + horizon_data['ref_vz'].values**2)

    ax4.plot(horizon_steps, pred_vel_mag, 'b.-', label='Predicted |V|', linewidth=2)
    ax4.plot(horizon_steps, ref_vel_mag, 'r--', label='Reference |V|', linewidth=2)

    # Add control visualization
    if is_torque_model:
        # For torque model, show thrust and torque magnitude
        torque_mag = np.sqrt(horizon_data['u1'].values[:-1]**2 +
                            horizon_data['u2'].values[:-1]**2 +
                            horizon_data['u3'].values[:-1]**2)
        ax4_twin = ax4.twinx()
        ax4_twin.plot(horizon_steps[:-1], torque_mag, 'm:', label='|τ| (Nm)', alpha=0.7, linewidth=1.5)
        ax4_twin.set_ylabel('Torque Magnitude (Nm)', color='m')
        ax4_twin.tick_params(axis='y', labelcolor='m')
        ax4_twin.legend(loc='upper right')
    else:
        # For rate model, show angular rate magnitude
        rate_mag = np.sqrt(horizon_data['u1'].values[:-1]**2 +
                          horizon_data['u2'].values[:-1]**2 +
                          horizon_data['u3'].values[:-1]**2)
        ax4_twin = ax4.twinx()
        ax4_twin.plot(horizon_steps[:-1], rate_mag, 'c:', label='|ω| (rad/s)', alpha=0.7, linewidth=1.5)
        ax4_twin.set_ylabel('Angular Rate Magnitude (rad/s)', color='c')
        ax4_twin.tick_params(axis='y', labelcolor='c')
        ax4_twin.legend(loc='upper right')

    ax4.plot(horizon_steps[:-1], horizon_data['thrust'].values[:-1]/10, 'k:', label='Thrust/10 (N)', alpha=0.5)

    ax4.set_xlabel('Horizon Step')
    ax4.set_ylabel('Velocity Magnitude (m/s)')
    ax4.set_title(f'Velocity & Control ({("Torque" if is_torque_model else "Rate")} Model)')
    ax4.legend(loc='upper left')
    ax4.grid(True, alpha=0.3)

    fig.suptitle(f'MPC Horizon Analysis at t={timestamp_to_plot:.2f}s\n'
                f'Comparing Reference Trajectory with MPC Predictions ({("Torque" if is_torque_model else "Rate")} Model)',
                fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()


def plot_mpc_horizon_evolution(horizon_df, states_df, time_window=(5.0, 10.0), save_path=None):
    """Plot how MPC predictions evolve over time compared to reference and actual"""
    if horizon_df is None:
        print("No horizon data available")
        return

    # Filter timestamps within the window
    unique_timestamps = horizon_df['timestamp'].unique()
    filtered_timestamps = unique_timestamps[
        (unique_timestamps >= time_window[0]) & (unique_timestamps <= time_window[1])
    ]

    if len(filtered_timestamps) == 0:
        print(f"No data in time window {time_window}")
        return

    fig = plt.figure(figsize=(16, 10))

    # 3D view showing multiple prediction horizons
    ax1 = fig.add_subplot(121, projection='3d')

    # Colormap for different timestamps
    colors = plt.cm.viridis(np.linspace(0, 1, len(filtered_timestamps)))

    for idx, timestamp in enumerate(filtered_timestamps[::2]):  # Plot every other for clarity
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]

        # Plot MPC predictions - Convert to numpy arrays
        ax1.plot(horizon_data['x'].values, horizon_data['y'].values, horizon_data['z'].values,
                color=colors[idx], alpha=0.6, linewidth=1.5,
                label=f't={timestamp:.1f}s' if idx % 2 == 0 else '')

        # Plot reference (only once) - Convert to numpy arrays
        if idx == 0:
            ax1.plot(horizon_data['ref_x'].values, horizon_data['ref_y'].values, horizon_data['ref_z'].values,
                    'r--', linewidth=2, label='Reference', alpha=0.8)

    # Plot actual trajectory from states_df if available
    if states_df is not None:
        actual_data = states_df[
            (states_df['timestamp'] >= time_window[0]) &
            (states_df['timestamp'] <= time_window[1])
        ]
        if not actual_data.empty:
            # Convert to numpy arrays
            ax1.plot(actual_data['x'].values, actual_data['y'].values, actual_data['z'].values,
                    'k-', linewidth=3, label='Actual', alpha=0.9)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Evolution of MPC Predictions Over Time')
    ax1.legend(fontsize=8, loc='upper left')
    ax1.grid(True, alpha=0.3)

    # 2D plot showing prediction accuracy over time
    ax2 = fig.add_subplot(122)

    # Calculate how prediction error grows with horizon
    horizon_errors = []
    horizon_steps = []

    for step in range(21):  # Assuming 20 horizon steps
        errors_at_step = []
        for timestamp in filtered_timestamps:
            horizon_data = horizon_df[
                (horizon_df['timestamp'] == timestamp) &
                (horizon_df['horizon_idx'] == step)
            ]
            if not horizon_data.empty:
                error = np.sqrt(
                    (horizon_data['x'].iloc[0] - horizon_data['ref_x'].iloc[0])**2 +
                    (horizon_data['y'].iloc[0] - horizon_data['ref_y'].iloc[0])**2 +
                    (horizon_data['z'].iloc[0] - horizon_data['ref_z'].iloc[0])**2
                )
                errors_at_step.append(error)

        if errors_at_step:
            horizon_errors.append(np.mean(errors_at_step))
            horizon_steps.append(step)

    ax2.plot(horizon_steps, horizon_errors, 'b.-', linewidth=2, markersize=8)
    ax2.fill_between(horizon_steps, horizon_errors, alpha=0.3)
    ax2.set_xlabel('Horizon Step')
    ax2.set_ylabel('Mean Prediction Error (m)')
    ax2.set_title('Prediction Error Growth Over Horizon')
    ax2.grid(True, alpha=0.3)

    # Add statistics
    if horizon_errors:
        ax2.text(0.98, 0.95,
                f'Error at step 0: {horizon_errors[0]:.4f} m\n'
                f'Error at step {len(horizon_errors)-1}: {horizon_errors[-1]:.4f} m\n'
                f'Error growth: {(horizon_errors[-1]/horizon_errors[0] - 1)*100:.1f}%',
                transform=ax2.transAxes, ha='right', va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    fig.suptitle(f'MPC Prediction Evolution Analysis\nTime Window: {time_window[0]:.1f}s to {time_window[1]:.1f}s',
                fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_consecutive_mpc_horizons(horizon_df, num_consecutive=3, timestamp_to_start=None, save_path=None):
    """Plot multiple consecutive MPC prediction horizons with their references and control commands"""
    if horizon_df is None:
        print("No horizon data available")
        return

    unique_timestamps = horizon_df['timestamp'].unique()

    # If no start timestamp specified, use one from the middle
    if timestamp_to_start is None:
        start_idx = len(unique_timestamps) // 2
    else:
        # Find the index of the requested timestamp
        start_idx = np.where(unique_timestamps == timestamp_to_start)[0]
        if len(start_idx) == 0:
            print(f"Timestamp {timestamp_to_start} not found")
            return
        start_idx = start_idx[0]

    # Make sure we have enough consecutive timestamps
    if start_idx + num_consecutive > len(unique_timestamps):
        start_idx = len(unique_timestamps) - num_consecutive

    # Get the consecutive timestamps
    timestamps_to_plot = unique_timestamps[start_idx:start_idx + num_consecutive]

    # Create figure with more subplots for controls
    fig = plt.figure(figsize=(24, 16))

    # Define colors for different timestamps using a colormap
    cmap_mpc = plt.cm.viridis(np.linspace(0.2, 0.8, num_consecutive))
    cmap_ref = plt.cm.autumn(np.linspace(0.2, 0.8, num_consecutive))

    colors = [cmap_mpc[i] for i in range(num_consecutive)]
    ref_colors = [cmap_ref[i] for i in range(num_consecutive)]

    # 3D view
    ax1 = fig.add_subplot(331, projection='3d')

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]

        # Plot MPC prediction
        ax1.plot(horizon_data['x'].values,
                horizon_data['y'].values,
                horizon_data['z'].values,
                color=colors[idx], alpha=0.7, linewidth=2,
                label=f'MPC t={timestamp:.2f}s', marker='.')

        # Plot reference trajectory
        ax1.plot(horizon_data['ref_x'].values,
                horizon_data['ref_y'].values,
                horizon_data['ref_z'].values,
                color=ref_colors[idx], alpha=0.7, linewidth=1.5,
                linestyle='--', marker='o', markersize=3,
                label=f'Ref t={timestamp:.2f}s')

        # Mark starting positions
        ax1.scatter(horizon_data['x'].iloc[0],
                   horizon_data['y'].iloc[0],
                   horizon_data['z'].iloc[0],
                   color=colors[idx], s=100, marker='*', edgecolor='black')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title(f'3D View: {num_consecutive} Consecutive MPC Horizons')
    ax1.legend(fontsize=6)
    ax1.grid(True, alpha=0.3)

    # XY plane view
    ax2 = fig.add_subplot(332)

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]

        # Plot MPC prediction
        ax2.plot(horizon_data['x'].values,
                horizon_data['y'].values,
                color=colors[idx], alpha=0.7, linewidth=2,
                label=f'MPC t={timestamp:.2f}s', marker='.')

        # Plot reference
        ax2.plot(horizon_data['ref_x'].values,
                horizon_data['ref_y'].values,
                color=ref_colors[idx], alpha=0.7, linewidth=1.5,
                linestyle='--', marker='o', markersize=3,
                label=f'Ref t={timestamp:.2f}s')

        # Mark start
        ax2.scatter(horizon_data['x'].iloc[0],
                   horizon_data['y'].iloc[0],
                   color=colors[idx], s=100, marker='*', edgecolor='black')

    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('XY Plane View')
    ax2.legend(fontsize=6)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    # Position errors over horizon for each timestamp
    ax3 = fig.add_subplot(333)

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]
        horizon_steps = horizon_data['horizon_idx'].values

        # Calculate error norm
        error_x = horizon_data['x'].values - horizon_data['ref_x'].values
        error_y = horizon_data['y'].values - horizon_data['ref_y'].values
        error_z = horizon_data['z'].values - horizon_data['ref_z'].values
        error_norm = np.sqrt(error_x**2 + error_y**2 + error_z**2)

        ax3.plot(horizon_steps, error_norm,
                color=colors[idx], linewidth=2, marker='.',
                label=f't={timestamp:.2f}s')

    ax3.set_xlabel('Horizon Step')
    ax3.set_ylabel('Position Error (m)')
    ax3.set_title('Tracking Error Over Horizon')
    ax3.legend(fontsize=6)
    ax3.grid(True, alpha=0.3)

    # THRUST Command over horizon
    ax4 = fig.add_subplot(334)

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]
        horizon_steps = horizon_data['horizon_idx'].values[:-1]  # Exclude terminal state

        # Plot thrust commands
        ax4.plot(horizon_steps, horizon_data['thrust'].values[:-1],
                color=colors[idx], linewidth=2, marker='o',
                label=f't={timestamp:.2f}s')

    ax4.set_xlabel('Horizon Step')
    ax4.set_ylabel('Thrust (N)')
    ax4.set_title('Thrust Commands Over Horizon')
    ax4.legend(fontsize=6)
    ax4.grid(True, alpha=0.3)

    # Detect if this is torque model or rate model based on angular velocity data
    is_torque_model = False
    if 'wx' in horizon_data.columns:
        # Check if any angular velocity data is non-zero
        sample_data = horizon_df[horizon_df['timestamp'] == timestamps_to_plot[0]]
        is_torque_model = (sample_data['wx'].abs().max() > 0 or
                          sample_data['wy'].abs().max() > 0 or
                          sample_data['wz'].abs().max() > 0)

    # Set labels based on model type
    if is_torque_model:
        u1_label = 'τx (Nm)'
        u2_label = 'τy (Nm)'
        u3_label = 'τz (Nm)'
        u1_title = 'Roll Torque Commands'
        u2_title = 'Pitch Torque Commands'
        u3_title = 'Yaw Torque Commands'
    else:
        u1_label = 'ωx (rad/s)'
        u2_label = 'ωy (rad/s)'
        u3_label = 'ωz (rad/s)'
        u1_title = 'Roll Rate Commands'
        u2_title = 'Pitch Rate Commands'
        u3_title = 'Yaw Rate Commands'

    # U1 Command over horizon
    ax5 = fig.add_subplot(335)

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]
        horizon_steps = horizon_data['horizon_idx'].values[:-1]

        # Plot u1 commands
        ax5.plot(horizon_steps, horizon_data['u1'].values[:-1],
                color=colors[idx], linewidth=2, marker='o',
                label=f't={timestamp:.2f}s')

    ax5.set_xlabel('Horizon Step')
    ax5.set_ylabel(u1_label)
    ax5.set_title(u1_title)
    ax5.legend(fontsize=6)
    ax5.grid(True, alpha=0.3)

    # U2 Command over horizon
    ax6 = fig.add_subplot(336)

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]
        horizon_steps = horizon_data['horizon_idx'].values[:-1]

        # Plot u2 commands
        ax6.plot(horizon_steps, horizon_data['u2'].values[:-1],
                color=colors[idx], linewidth=2, marker='o',
                label=f't={timestamp:.2f}s')

    ax6.set_xlabel('Horizon Step')
    ax6.set_ylabel(u2_label)
    ax6.set_title(u2_title)
    ax6.legend(fontsize=6)
    ax6.grid(True, alpha=0.3)

    # U3 Command over horizon
    ax7 = fig.add_subplot(337)

    for idx, timestamp in enumerate(timestamps_to_plot):
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]
        horizon_steps = horizon_data['horizon_idx'].values[:-1]

        # Plot u3 commands
        ax7.plot(horizon_steps, horizon_data['u3'].values[:-1],
                color=colors[idx], linewidth=2, marker='o',
                label=f't={timestamp:.2f}s')

    ax7.set_xlabel('Horizon Step')
    ax7.set_ylabel(u3_label)
    ax7.set_title(u3_title)
    ax7.legend(fontsize=6)
    ax7.grid(True, alpha=0.3)

    # Show how first control action changes between consecutive solutions
    ax8 = fig.add_subplot(338)

    first_thrust = []
    first_u1 = []
    first_u2 = []
    first_u3 = []

    for timestamp in timestamps_to_plot:
        horizon_data = horizon_df[horizon_df['timestamp'] == timestamp]
        # Get control at step 0
        ctrl_at_0 = horizon_data[horizon_data['horizon_idx'] == 0]
        if not ctrl_at_0.empty:
            first_thrust.append(ctrl_at_0['thrust'].iloc[0])
            first_u1.append(ctrl_at_0['u1'].iloc[0])
            first_u2.append(ctrl_at_0['u2'].iloc[0])
            first_u3.append(ctrl_at_0['u3'].iloc[0])

    time_steps = np.arange(len(first_thrust))
    ax8.plot(time_steps, first_thrust, 'k.-', label='Thrust', linewidth=2)
    ax8.set_xlabel('Time Step')
    ax8.set_ylabel('First Control Action')
    ax8.set_title('Evolution of First Control Action (Thrust)')
    ax8.set_xticks(time_steps)
    if num_consecutive <= 5:
        ax8.set_xticklabels([f't={t:.2f}' for t in timestamps_to_plot], rotation=45)
    else:
        # For many timestamps, show fewer labels
        tick_indices = np.linspace(0, num_consecutive-1, min(5, num_consecutive), dtype=int)
        ax8.set_xticks(tick_indices)
        ax8.set_xticklabels([f't={timestamps_to_plot[i]:.2f}' for i in tick_indices], rotation=45)
    ax8.legend()
    ax8.grid(True, alpha=0.3)

    # Show how first rate/torque commands change between consecutive solutions
    ax9 = fig.add_subplot(339)

    ax9.plot(time_steps, first_u1, 'r.-', label=f'U1 ({u1_label})', linewidth=2)
    ax9.plot(time_steps, first_u2, 'g.-', label=f'U2 ({u2_label})', linewidth=2)
    ax9.plot(time_steps, first_u3, 'b.-', label=f'U3 ({u3_label})', linewidth=2)

    ax9.set_xlabel('Time Step')
    if is_torque_model:
        ax9.set_ylabel('First Torque Commands')
        ax9.set_title('Evolution of First Torque Commands')
    else:
        ax9.set_ylabel('First Rate Commands')
        ax9.set_title('Evolution of First Rate Commands')
    ax9.set_xticks(time_steps)
    if num_consecutive <= 5:
        ax9.set_xticklabels([f't={t:.2f}' for t in timestamps_to_plot], rotation=45)
    else:
        # For many timestamps, show fewer labels
        tick_indices = np.linspace(0, num_consecutive-1, min(5, num_consecutive), dtype=int)
        ax9.set_xticks(tick_indices)
        ax9.set_xticklabels([f't={timestamps_to_plot[i]:.2f}' for i in tick_indices], rotation=45)
    ax9.legend()
    ax9.grid(True, alpha=0.3)

    # Add analysis text showing command variations
    if len(first_u1) > 1:
        u1_var = np.std(first_u1)
        u2_var = np.std(first_u2)
        u3_var = np.std(first_u3)
        thrust_var = np.std(first_thrust)

        if is_torque_model:
            unit_str = 'Nm'
        else:
            unit_str = 'rad/s'

        analysis_text = (f'Command Variations (std dev):\n'
                        f'Thrust: {thrust_var:.3f} N\n'
                        f'U1: {u1_var:.4f} {unit_str}\n'
                        f'U2: {u2_var:.4f} {unit_str}\n'
                        f'U3: {u3_var:.4f} {unit_str}')

        # Add text box to last subplot
        ax9.text(0.02, 0.98, analysis_text,
                transform=ax9.transAxes, ha='left', va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                fontsize=8)

    fig.suptitle(f'Consecutive MPC Horizons with Control Commands Analysis\n'
                f'Timestamps: {timestamps_to_plot[0]:.2f}s to {timestamps_to_plot[-1]:.2f}s',
                fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Plot MPC trajectory tracking data')
    parser.add_argument('log_directory', type=str, nargs='?',
                       default='/tmp/mpc_logs/latest',
                       help='Path to log directory containing CSV files (default: /tmp/mpc_logs/latest)')
    parser.add_argument('--save-dir', type=str, default=None,
                       help='Directory to save plots (optional)')
    parser.add_argument('--show-horizon', action='store_true',
                       help='Show MPC horizon predictions')
    parser.add_argument('--show-consecutive', action='store_true',
                       help='Show consecutive MPC horizons')
    parser.add_argument('--all', action='store_true',
                       help='Show combined plot with all states and controls')
    parser.add_argument('--estimate-inertia', action='store_true',
                       help='Estimate inertia matrix and actuator dynamics from torque model data')
    parser.add_argument('--max-tau-c', type=float, default=0.1, dest='max_tau_c',
                       help='Maximum actuator time constant in seconds to test (default: 0.1s = 100ms)')

    args = parser.parse_args()

    # Load data
    print(f"Loading data from: {args.log_directory}")
    states_df, horizon_df = load_data(args.log_directory)

    print(f"Loaded {len(states_df)} state samples")
    if horizon_df is not None:
        print(f"Loaded {len(horizon_df)} horizon samples")

    # Check controller type based on angular velocity data
    controller_type = "Unknown"
    is_torque_model = False
    if 'wx' in states_df.columns:
        has_angular_vel = states_df['wx'].abs().max() > 0 or states_df['wy'].abs().max() > 0 or states_df['wz'].abs().max() > 0
        if has_angular_vel:
            controller_type = "TORQUE"
            is_torque_model = True
            print(f"\n=== Controller Type Detected: TORQUE Model ===")
            print("  - Control commands (u1, u2, u3) are torques in Newton-meters (Nm)")
            print("  - State includes angular velocities (ωx, ωy, ωz) in rad/s")
        else:
            controller_type = "RATE"
            print(f"\n=== Controller Type Detected: RATE Model ===")
            print("  - Control commands (u1, u2, u3) are angular rates in rad/s")
            print("  - No angular velocity states")
    else:
        controller_type = "RATE"
        print(f"\n=== Controller Type: RATE Model (old log format) ===")
        print("  - Control commands (u1, u2, u3) are angular rates in rad/s")

    # Run inertia estimation if requested and torque model is detected
    if args.estimate_inertia:
        if is_torque_model:
            inertia_result = estimate_inertia_matrix(states_df, max_tau_c=args.max_tau_c)
            if inertia_result:
                print("\n=== Inertia Estimation Results Summary ===")
                print(f"Ixx: {inertia_result['Ixx']:.6f} kg⋅m²")
                print(f"Iyy: {inertia_result['Iyy']:.6f} kg⋅m²")
                print(f"Izz: {inertia_result['Izz']:.6f} kg⋅m²")
                print(f"Actuator time constant: {inertia_result['tau_c']:.4f} s ({inertia_result['tau_c_ms']:.1f} ms)")
                print(f"Actuator bandwidth: {inertia_result['bandwidth_hz']:.1f} Hz")
                print(f"Goodness of fit (R²): X={inertia_result['r2_x']:.3f}, Y={inertia_result['r2_y']:.3f}, Z={inertia_result['r2_z']:.3f}")
        else:
            print("\n⚠ Inertia estimation requires torque model data with angular velocities.")
            print("  The current log appears to be from a rate model controller.")
        return  # Exit after inertia estimation

    # Create save directory if specified
    save_dir = None
    if args.save_dir:
        save_dir = Path(args.save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)

    # Generate plots
    print("\nGenerating plots...")

    # Show combined plot if requested
    if args.all:
        save_path = save_dir / "all_combined.png" if save_dir else None
        plot_all_combined(states_df, horizon_df, save_path)
        return  # Exit after showing the combined plot

    # Otherwise show individual plots as before
    # Position tracking
    save_path = save_dir / "position_tracking.png" if save_dir else None
    plot_position_tracking(states_df, save_path)

    # 3D trajectory
    save_path = save_dir / "3d_trajectory.png" if save_dir else None
    plot_3d_trajectory(states_df, save_path)

    # Tracking errors
    save_path = save_dir / "tracking_errors.png" if save_dir else None
    plot_tracking_errors(states_df, save_path)

    # Control inputs
    save_path = save_dir / "control_inputs.png" if save_dir else None
    plot_control_inputs(states_df, save_path)

    # Velocity tracking
    save_path = save_dir / "velocity_tracking.png" if save_dir else None
    plot_velocity_tracking(states_df, save_path)

    # MPC horizon comparison (if requested and available)
    if args.show_horizon and horizon_df is not None:
        save_path = save_dir / "mpc_horizon_comparison.png" if save_dir else None
        plot_mpc_horizon_comparison(horizon_df, save_path=save_path)

        # Also plot evolution analysis
        save_path = save_dir / "mpc_horizon_evolution.png" if save_dir else None
        plot_mpc_horizon_evolution(horizon_df, states_df, time_window=(5.0, 15.0), save_path=save_path)

    # Plot consecutive MPC horizons
    if args.show_consecutive and horizon_df is not None:
        save_path = save_dir / "consecutive_mpc_horizons.png" if save_dir else None
        plot_consecutive_mpc_horizons(horizon_df, num_consecutive=6, save_path=save_path)

    # Print statistics
    print("\n=== Tracking Performance Statistics ===")
    print(f"Mean position error: {states_df['error_norm'].mean():.4f} m")
    print(f"Max position error: {states_df['error_norm'].max():.4f} m")
    print(f"Std position error: {states_df['error_norm'].std():.4f} m")
    print(f"Final position error: {states_df['error_norm'].iloc[-1]:.4f} m")

    # Detect controller type
    is_torque_model = ('wx' in states_df.columns) and (
        states_df['wx'].abs().max() > 0 or
        states_df['wy'].abs().max() > 0 or
        states_df['wz'].abs().max() > 0)

    print("\n=== Control Effort Statistics ===")
    print(f"Controller Type: {'TORQUE Model' if is_torque_model else 'RATE Model'}")
    print(f"Mean thrust: {states_df['thrust'].mean():.2f} N")
    print(f"Max thrust: {states_df['thrust'].max():.2f} N")
    print(f"Min thrust: {states_df['thrust'].min():.2f} N")

    if is_torque_model:
        print(f"\nTorque Commands:")
        print(f"τx (Roll):  Mean={states_df['u1'].mean():.4f} Nm, Max={states_df['u1'].abs().max():.4f} Nm, Std={states_df['u1'].std():.4f} Nm")
        print(f"τy (Pitch): Mean={states_df['u2'].mean():.4f} Nm, Max={states_df['u2'].abs().max():.4f} Nm, Std={states_df['u2'].std():.4f} Nm")
        print(f"τz (Yaw):   Mean={states_df['u3'].mean():.4f} Nm, Max={states_df['u3'].abs().max():.4f} Nm, Std={states_df['u3'].std():.4f} Nm")

        print("\n=== Angular Velocity Statistics ===")
        print(f"ωx (Roll):  Mean={states_df['wx'].mean():.4f} rad/s, Max={states_df['wx'].abs().max():.4f} rad/s")
        print(f"ωy (Pitch): Mean={states_df['wy'].mean():.4f} rad/s, Max={states_df['wy'].abs().max():.4f} rad/s")
        print(f"ωz (Yaw):   Mean={states_df['wz'].mean():.4f} rad/s, Max={states_df['wz'].abs().max():.4f} rad/s")
    else:
        print(f"\nRate Commands:")
        print(f"ωx (Roll):  Mean={states_df['u1'].mean():.4f} rad/s, Max={states_df['u1'].abs().max():.4f} rad/s, Std={states_df['u1'].std():.4f} rad/s")
        print(f"ωy (Pitch): Mean={states_df['u2'].mean():.4f} rad/s, Max={states_df['u2'].abs().max():.4f} rad/s, Std={states_df['u2'].std():.4f} rad/s")
        print(f"ωz (Yaw):   Mean={states_df['u3'].mean():.4f} rad/s, Max={states_df['u3'].abs().max():.4f} rad/s, Std={states_df['u3'].std():.4f} rad/s")

if __name__ == '__main__':
    main()
