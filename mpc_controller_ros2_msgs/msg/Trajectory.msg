# MPC Trajectory message
# Contains a time-stamped sequence of trajectory states

# Header with timestamp
std_msgs/Header header

# Trajectory start time (seconds since epoch)
# This is the absolute time when the trajectory starts
float64 t_0

# Time discretization step (seconds)
# Time between consecutive states in the trajectory
float64 dt

# Sequence of trajectory states
# Each state is dt seconds apart
# states[i] occurs at time: t_0 + i * dt
TrajectoryState[] states
