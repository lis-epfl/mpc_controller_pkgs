# MPC Controller ROS2 Messages

ROS2 message definitions for MPC trajectory planning and control.

## Messages

### TrajectoryState.msg

Represents a single state in the trajectory, containing:
- **position** (`geometry_msgs/Point`) - 3D position (x, y, z) in meters
- **orientation** (`geometry_msgs/Quaternion`) - Orientation as quaternion (w, x, y, z)
- **velocity** (`geometry_msgs/Vector3`) - Linear velocity (vx, vy, vz) in m/s
- **angular_velocity** (`geometry_msgs/Vector3`) - Angular velocity (wx, wy, wz) in rad/s

### Trajectory.msg

Represents a complete MPC trajectory, containing:
- **header** (`std_msgs/Header`) - Standard ROS header with timestamp
- **t_0** (`float64`) - Trajectory start time (seconds since epoch)
- **dt** (`float64`) - Time discretization step (seconds between states)
- **states** (`TrajectoryState[]`) - Array of trajectory states

The time for state `i` is calculated as: `t_i = t_0 + i * dt`

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select mpc_controller_ros2_msgs
source install/setup.bash
```

## Usage

**Python:**
```python
from mpc_controller_ros2_msgs.msg import Trajectory, TrajectoryState
from geometry_msgs.msg import Point, Quaternion, Vector3

msg = Trajectory()
msg.t_0 = time.time()
msg.dt = 0.1

state = TrajectoryState()
state.position = Point(x=1.0, y=0.0, z=1.0)
state.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
state.velocity = Vector3(x=0.5, y=0.0, z=0.0)
state.angular_velocity = Vector3(x=0.0, y=0.0, z=0.1)
msg.states.append(state)
```

**C++:**
```cpp
#include <mpc_controller_ros2_msgs/msg/trajectory.hpp>
#include <mpc_controller_ros2_msgs/msg/trajectory_state.hpp>

auto msg = mpc_controller_ros2_msgs::msg::Trajectory();
msg.t_0 = this->get_clock()->now().seconds();
msg.dt = 0.1;

auto state = mpc_controller_ros2_msgs::msg::TrajectoryState();
state.position.x = 1.0;
// ... set other fields
msg.states.push_back(state);
```

## Message Inspection

```bash
ros2 interface show mpc_controller_ros2_msgs/msg/Trajectory
ros2 topic echo /mpc/trajectory
```
