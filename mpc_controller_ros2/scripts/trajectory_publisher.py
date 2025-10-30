#!/usr/bin/env python3
"""
Publish a trajectory for the MPC controller to follow
"""

import rclpy
from rclpy.node import Node
from mpc_controller_ros2_msgs.msg import Trajectory, TrajectoryState
from geometry_msgs.msg import Point, Quaternion, Vector3
import numpy as np
import math
import argparse

class TrajectoryPublisher(Node):
    def __init__(self, args=None):
        super().__init__('trajectory_publisher')

        # Set defaults
        default_values = {
            'initial_position': [0.0, 0.0, 1.0],
            'radius': 2.0,
            'pitch': 1.0,
            'speed': 1.0,
            'n_revolutions': 3.0,
            'traj_dt': 0.1,
            'horizon': 3.0,
            'publish_rate': 100.0,
            'yaw': 0.0,
            'hover_time': 2.0,
            'trajectory_type': 'helix',
            'topic': '/planner/trajectory',
            'transition_time': 3.0  # Time to join the trajectory
        }

        # Override with command line args if provided
        if args:
            self.initial_pos = [args.x0, args.y0, args.z0]
            self.radius = args.radius
            self.pitch = args.pitch
            self.speed = args.speed
            self.n_revolutions = args.revolutions
            self.traj_dt = default_values['traj_dt']
            self.horizon = default_values['horizon']
            self.publish_rate = default_values['publish_rate']
            self.yaw = args.yaw
            self.hover_time = args.hover_time
            self.trajectory_type = args.type
            self.topic = args.topic
            self.transition_time = args.transition_time
        else:
            # Use defaults
            self.initial_pos = default_values['initial_position']
            self.radius = default_values['radius']
            self.pitch = default_values['pitch']
            self.speed = default_values['speed']
            self.n_revolutions = default_values['n_revolutions']
            self.traj_dt = default_values['traj_dt']
            self.horizon = default_values['horizon']
            self.publish_rate = default_values['publish_rate']
            self.yaw = default_values['yaw']
            self.hover_time = default_values['hover_time']
            self.trajectory_type = default_values['trajectory_type']
            self.topic = default_values['topic']
            self.transition_time = default_values['transition_time']

        # Publisher
        self.trajectory_pub = self.create_publisher(
            Trajectory, self.topic, 10)

        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

        # State
        self.start_time = self.get_clock().now()
        self.trajectory_started = False

        # Compute the starting point and velocity on the trajectory
        self.trajectory_start_point, self.trajectory_start_velocity = self.get_trajectory_start_conditions()

        self.get_logger().info(f'Trajectory Publisher initialized')
        self.get_logger().info(f'Publishing to topic: {self.topic}')
        self.get_logger().info(f'Type: {self.trajectory_type}')
        self.get_logger().info(f'Initial position: {self.initial_pos}')
        self.get_logger().info(f'Trajectory start point: {self.trajectory_start_point}')
        self.get_logger().info(f'Trajectory start velocity: {self.trajectory_start_velocity}')
        self.get_logger().info(f'Radius: {self.radius} m, Pitch: {self.pitch} m/rev')
        self.get_logger().info(f'Speed: {self.speed} m/s, Revolutions: {self.n_revolutions}')
        self.get_logger().info(f'Hover time before trajectory: {self.hover_time} s')
        self.get_logger().info(f'Transition time to join trajectory: {self.transition_time} s')

        # Log expected completion time
        if self.trajectory_type == 'helix':
            distance_per_rev = np.sqrt((2 * np.pi * self.radius)**2 + self.pitch**2)
            time_per_rev = distance_per_rev / self.speed
            total_time = time_per_rev * self.n_revolutions
            self.get_logger().info(f'Helix distance per revolution: {distance_per_rev:.2f} m')
            self.get_logger().info(f'Expected trajectory time: {total_time:.2f} s (after transition)')
            self.get_logger().info(f'Final altitude will be: {self.initial_pos[2] + self.pitch * self.n_revolutions:.2f} m')

    def get_trajectory_start_conditions(self):
        """Get the starting point and velocity on the trajectory (at theta=0)"""
        if self.trajectory_type == 'helix':
            distance_per_revolution = np.sqrt((2 * np.pi * self.radius)**2 + self.pitch**2)
            time_per_revolution = distance_per_revolution / self.speed
            angular_rate = 2 * np.pi / time_per_revolution
            vertical_rate = self.pitch / time_per_revolution

            position = [
                self.initial_pos[0] + self.radius,
                self.initial_pos[1],
                self.initial_pos[2]
            ]
            velocity = [
                0.0,
                self.radius * angular_rate,
                vertical_rate
            ]

        elif self.trajectory_type == 'circle':
            angular_rate = self.speed / self.radius

            position = [
                self.initial_pos[0] + self.radius,
                self.initial_pos[1],
                self.initial_pos[2]
            ]
            velocity = [
                0.0,
                self.radius * angular_rate,
                0.0
            ]

        elif self.trajectory_type == 'lemniscate':
            # Lemniscate is centered at initial_pos and starts from initial_pos
            # We'll start at theta such that the lemniscate passes through origin (center)
            # At theta = pi/2: x = 0, y = 0 (center of figure-8)
            a = self.radius
            angular_rate = self.speed / (2 * a)

            # Start at center (initial position)
            position = [
                self.initial_pos[0],
                self.initial_pos[1],
                self.initial_pos[2]
            ]
            # At theta = pi/2: dx/dtheta = -a, dy/dtheta = -a
            velocity = [
                -a * angular_rate,
                -a * angular_rate,
                0.0
            ]

        elif self.trajectory_type == 'square':
            # First waypoint to second waypoint direction
            position = [
                self.initial_pos[0] + self.radius,
                self.initial_pos[1],
                self.initial_pos[2]
            ]
            velocity = [
                self.speed,
                0.0,
                0.0
            ]
        else:
            position = self.initial_pos
            velocity = [0.0, 0.0, 0.0]

        return position, velocity

    def smooth_transition(self, t_norm):
        """
        Smooth transition function using a quintic polynomial (5th order)
        Returns a value between 0 and 1 that smoothly transitions
        with specified velocity and acceleration at endpoints
        """
        if t_norm <= 0:
            return 0.0
        elif t_norm >= 1:
            return 1.0
        else:
            # Quintic: 6t^5 - 15t^4 + 10t^3
            return 6 * t_norm**5 - 15 * t_norm**4 + 10 * t_norm**3

    def generate_transition_state(self, t_transition):
        """
        Generate state during transition from initial position to trajectory start
        Uses quintic polynomial to match position, velocity, and acceleration at both endpoints

        Boundary conditions:
        - t=0: p=start_pos, v=start_vel=[0,0,0], a=0
        - t=T: p=end_pos, v=end_vel, a=0
        """
        T = self.transition_time
        t = t_transition

        start_pos = np.array(self.initial_pos)
        end_pos = np.array(self.trajectory_start_point)
        start_vel = np.array([0.0, 0.0, 0.0])
        end_vel = np.array(self.trajectory_start_velocity)

        # Quintic polynomial coefficients for each dimension
        # p(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
        # Given boundary conditions:
        # c0 = p0
        # c1 = v0
        # c2 = 0 (a0 = 0)
        # Solving the system for c3, c4, c5:

        position = np.zeros(3)
        velocity = np.zeros(3)

        for i in range(3):
            p0 = start_pos[i]
            pT = end_pos[i]
            v0 = start_vel[i]
            vT = end_vel[i]

            # Quintic coefficients
            c0 = p0
            c1 = v0
            c2 = 0.0  # acceleration at start is zero

            # Solve for c3, c4, c5 from boundary conditions at t=T
            # pT = c0 + c1*T + c2*T² + c3*T³ + c4*T⁴ + c5*T⁵
            # vT = c1 + 2*c2*T + 3*c3*T² + 4*c4*T³ + 5*c5*T⁴
            # aT = 2*c2 + 6*c3*T + 12*c4*T² + 20*c5*T³ = 0

            # Matrix form: [T³  T⁴  T⁵ ] [c3]   [pT - c0 - c1*T]
            #              [3T² 4T³ 5T⁴] [c4] = [vT - c1      ]
            #              [6T  12T² 20T³] [c5]   [0           ]

            A = np.array([
                [T**3, T**4, T**5],
                [3*T**2, 4*T**3, 5*T**4],
                [6*T, 12*T**2, 20*T**3]
            ])
            b = np.array([
                pT - c0 - c1*T,
                vT - c1,
                0.0
            ])

            c3, c4, c5 = np.linalg.solve(A, b)

            # Evaluate position and velocity at time t
            position[i] = c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5
            velocity[i] = c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4

        state = TrajectoryState()
        state.position = Point(x=position[0], y=position[1], z=position[2])
        state.velocity = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
        state.orientation = self.yaw_to_quaternion(self.yaw)
        state.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)

        return state

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (rotation around z-axis)"""
        return Quaternion(
            w=math.cos(yaw / 2.0),
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0)
        )

    def generate_helix_trajectory(self, t_start, duration):
        """Generate helix trajectory points"""
        n_points = int(duration / self.traj_dt) + 1
        times = np.linspace(0, duration, n_points)

        states = []

        # Helix parameters
        distance_per_revolution = np.sqrt((2 * np.pi * self.radius)**2 + self.pitch**2)
        time_per_revolution = distance_per_revolution / self.speed
        angular_rate = 2 * np.pi / time_per_revolution
        vertical_rate = self.pitch / time_per_revolution
        time_for_revolutions = time_per_revolution * self.n_revolutions

        for t in times:
            state = TrajectoryState()
            t_traj = t_start + t

            if t_traj < 0:
                # Haven't started, hover at initial position
                state.position = Point(
                    x=self.initial_pos[0],
                    y=self.initial_pos[1],
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            elif t_traj < self.transition_time:
                # Transition phase
                state = self.generate_transition_state(t_traj)
            elif t_traj - self.transition_time >= time_for_revolutions:
                # Completed all revolutions, hover at final position
                theta_final = angular_rate * time_for_revolutions
                state.position = Point(
                    x=self.initial_pos[0] + self.radius * np.cos(theta_final),
                    y=self.initial_pos[1] + self.radius * np.sin(theta_final),
                    z=self.initial_pos[2] + vertical_rate * time_for_revolutions
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                # Following helix trajectory
                t_helix = t_traj - self.transition_time
                theta = angular_rate * t_helix
                state.position = Point(
                    x=self.initial_pos[0] + self.radius * np.cos(theta),
                    y=self.initial_pos[1] + self.radius * np.sin(theta),
                    z=self.initial_pos[2] + vertical_rate * t_helix
                )
                state.velocity = Vector3(
                    x=-self.radius * angular_rate * np.sin(theta),
                    y=self.radius * angular_rate * np.cos(theta),
                    z=vertical_rate
                )

            state.orientation = self.yaw_to_quaternion(self.yaw)
            state.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
            states.append(state)

        return states

    def generate_circle_trajectory(self, t_start, duration):
        """Generate circular trajectory at constant altitude"""
        n_points = int(duration / self.traj_dt) + 1
        times = np.linspace(0, duration, n_points)

        states = []

        # Circle parameters
        angular_rate = self.speed / self.radius
        time_for_revolutions = (2 * np.pi * self.n_revolutions) / angular_rate

        for t in times:
            state = TrajectoryState()
            t_traj = t_start + t

            if t_traj < 0:
                state.position = Point(
                    x=self.initial_pos[0],
                    y=self.initial_pos[1],
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            elif t_traj < self.transition_time:
                # Transition phase
                state = self.generate_transition_state(t_traj)
            elif t_traj - self.transition_time >= time_for_revolutions:
                theta_final = angular_rate * time_for_revolutions
                state.position = Point(
                    x=self.initial_pos[0] + self.radius * np.cos(theta_final),
                    y=self.initial_pos[1] + self.radius * np.sin(theta_final),
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                t_circle = t_traj - self.transition_time
                theta = angular_rate * t_circle
                state.position = Point(
                    x=self.initial_pos[0] + self.radius * np.cos(theta),
                    y=self.initial_pos[1] + self.radius * np.sin(theta),
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(
                    x=-self.radius * angular_rate * np.sin(theta),
                    y=self.radius * angular_rate * np.cos(theta),
                    z=0.0
                )

            state.orientation = self.yaw_to_quaternion(self.yaw)
            state.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
            states.append(state)

        return states

    def generate_lemniscate_trajectory(self, t_start, duration):
        """Generate figure-8 (lemniscate) trajectory starting from center"""
        n_points = int(duration / self.traj_dt) + 1
        times = np.linspace(0, duration, n_points)

        states = []

        # Lemniscate parameters
        a = self.radius
        angular_rate = self.speed / (2 * a)
        time_for_one_loop = (2 * np.pi) / angular_rate
        time_for_revolutions = time_for_one_loop * self.n_revolutions

        # Start at theta = pi/2 so the lemniscate begins at the center (initial_pos)
        theta_offset = np.pi / 2

        for t in times:
            state = TrajectoryState()
            t_traj = t_start + t

            if t_traj < 0:
                state.position = Point(
                    x=self.initial_pos[0],
                    y=self.initial_pos[1],
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            elif t_traj < self.transition_time:
                # Transition phase
                state = self.generate_transition_state(t_traj)
            elif t_traj - self.transition_time >= time_for_revolutions:
                theta_final = angular_rate * time_for_revolutions + theta_offset
                denominator = 1 + np.sin(theta_final)**2
                x_final = a * np.cos(theta_final) / denominator
                y_final = a * np.sin(theta_final) * np.cos(theta_final) / denominator

                state.position = Point(
                    x=self.initial_pos[0] + x_final,
                    y=self.initial_pos[1] + y_final,
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                t_lemn = t_traj - self.transition_time
                theta = angular_rate * t_lemn + theta_offset
                denominator = 1 + np.sin(theta)**2
                x = a * np.cos(theta) / denominator
                y = a * np.sin(theta) * np.cos(theta) / denominator

                dx_dtheta = -a * np.sin(theta) * (1 + np.sin(theta)**2 + 2*np.cos(theta)**2) / (denominator**2)
                dy_dtheta = a * np.cos(theta) * (np.cos(theta)**2 - np.sin(theta)**2) / (denominator**2)

                state.position = Point(
                    x=self.initial_pos[0] + x,
                    y=self.initial_pos[1] + y,
                    z=self.initial_pos[2]
                )
                state.velocity = Vector3(
                    x=dx_dtheta * angular_rate,
                    y=dy_dtheta * angular_rate,
                    z=0.0
                )

            state.orientation = self.yaw_to_quaternion(self.yaw)
            state.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
            states.append(state)

        return states

    def generate_waypoint_trajectory(self, waypoints, t_start, duration):
        """Generate trajectory through waypoints with smooth interpolation"""
        n_points = int(duration / self.traj_dt) + 1
        times = np.linspace(0, duration, n_points)

        states = []

        total_distance = 0
        distances = [0]
        for i in range(1, len(waypoints)):
            dist = np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i-1]))
            total_distance += dist
            distances.append(total_distance)

        time_for_one_loop = total_distance / self.speed
        time_for_revolutions = time_for_one_loop * self.n_revolutions

        for t in times:
            state = TrajectoryState()
            t_traj = t_start + t

            if t_traj < 0:
                state.position = Point(
                    x=waypoints[0][0],
                    y=waypoints[0][1],
                    z=waypoints[0][2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            elif t_traj < self.transition_time:
                # Transition phase
                state = self.generate_transition_state(t_traj)
            elif t_traj - self.transition_time >= time_for_revolutions:
                state.position = Point(
                    x=waypoints[-1][0],
                    y=waypoints[-1][1],
                    z=waypoints[-1][2]
                )
                state.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                t_waypoint = t_traj - self.transition_time
                distance_traveled = (self.speed * t_waypoint) % total_distance

                for i in range(len(distances)-1):
                    if distances[i] <= distance_traveled <= distances[i+1]:
                        segment_progress = (distance_traveled - distances[i]) / (distances[i+1] - distances[i])

                        p0 = np.array(waypoints[i])
                        p1 = np.array(waypoints[i+1])
                        position = p0 + segment_progress * (p1 - p0)

                        direction = (p1 - p0) / np.linalg.norm(p1 - p0)
                        velocity = self.speed * direction

                        state.position = Point(x=position[0], y=position[1], z=position[2])
                        state.velocity = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
                        break

            state.orientation = self.yaw_to_quaternion(self.yaw)
            state.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
            states.append(state)

        return states

    def publish_trajectory(self):
        """Publish trajectory message"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        t_start = elapsed - self.hover_time

        if not self.trajectory_started and t_start >= 0:
            self.trajectory_started = True
            self.trajectory_start_time = current_time.nanoseconds / 1e9 - t_start
            self.get_logger().info('Starting trajectory execution')

        # Generate trajectory based on type
        if self.trajectory_type == 'helix':
            states = self.generate_helix_trajectory(t_start, self.horizon)
        elif self.trajectory_type == 'circle':
            states = self.generate_circle_trajectory(t_start, self.horizon)
        elif self.trajectory_type == 'lemniscate':
            states = self.generate_lemniscate_trajectory(t_start, self.horizon)
        elif self.trajectory_type == 'square':
            s = self.radius
            waypoints = [
                self.initial_pos,
                [self.initial_pos[0] + s, self.initial_pos[1], self.initial_pos[2]],
                [self.initial_pos[0] + s, self.initial_pos[1] + s, self.initial_pos[2]],
                [self.initial_pos[0], self.initial_pos[1] + s, self.initial_pos[2]],
                self.initial_pos
            ]
            states = self.generate_waypoint_trajectory(waypoints, t_start, self.horizon)
        else:
            self.get_logger().error(f'Unknown trajectory type: {self.trajectory_type}')
            return

        # Create and publish message
        msg = Trajectory()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'world'
        msg.t_0 = current_time.nanoseconds / 1e9
        msg.dt = self.traj_dt
        msg.states = states

        self.trajectory_pub.publish(msg)

        # Log status occasionally
        if int(elapsed * 10) % 10 == 0:
            if t_start < 0:
                self.get_logger().info(f'Hovering for {-t_start:.1f} more seconds...',
                                      throttle_duration_sec=1.0)
            elif t_start < self.transition_time:
                progress = (t_start / self.transition_time) * 100
                self.get_logger().info(f'Transitioning to trajectory: {progress:.1f}%',
                                      throttle_duration_sec=1.0)
            else:
                # Calculate completion time based on trajectory type
                t_traj = t_start - self.transition_time

                if self.trajectory_type == 'helix':
                    distance_per_revolution = np.sqrt((2 * np.pi * self.radius)**2 + self.pitch**2)
                    time_per_revolution = distance_per_revolution / self.speed
                    time_for_revolutions = time_per_revolution * self.n_revolutions
                elif self.trajectory_type == 'circle':
                    angular_rate = self.speed / self.radius
                    time_for_revolutions = (2 * np.pi * self.n_revolutions) / angular_rate
                elif self.trajectory_type == 'lemniscate':
                    angular_rate = self.speed / (2 * self.radius)
                    time_for_one_loop = (2 * np.pi) / angular_rate
                    time_for_revolutions = time_for_one_loop * self.n_revolutions
                elif self.trajectory_type == 'square':
                    perimeter = 4 * self.radius
                    time_for_revolutions = (perimeter * self.n_revolutions) / self.speed
                else:
                    time_for_revolutions = float('inf')

                if t_traj >= time_for_revolutions:
                    self.get_logger().info(f'{self.trajectory_type.capitalize()} complete! Hovering at final position.',
                                        throttle_duration_sec=1.0)
                else:
                    progress = (t_traj / time_for_revolutions) * 100
                    self.get_logger().info(f'{self.trajectory_type.capitalize()} progress: {progress:.1f}%',
                                        throttle_duration_sec=1.0)

def main(args=None):
    parser = argparse.ArgumentParser(description='Trajectory publisher for MPC controller')
    parser.add_argument('--type', type=str, default='helix',
                       choices=['helix', 'circle', 'lemniscate', 'square'],
                       help='Type of trajectory to generate')
    parser.add_argument('--x0', type=float, default=0.0, help='Initial X position')
    parser.add_argument('--y0', type=float, default=0.0, help='Initial Y position')
    parser.add_argument('--z0', type=float, default=1.0, help='Initial Z position')
    parser.add_argument('--radius', type=float, default=2.0, help='Radius of trajectory')
    parser.add_argument('--pitch', type=float, default=1.0, help='Helix pitch (m/revolution)')
    parser.add_argument('--speed', type=float, default=1.0, help='Tangential speed (m/s)')
    parser.add_argument('--revolutions', type=float, default=3.0, help='Number of revolutions')
    parser.add_argument('--hover-time', type=float, default=2.0, help='Hover time before trajectory')
    parser.add_argument('--transition-time', type=float, default=3.0, help='Time to smoothly join trajectory')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw angle in radians')
    parser.add_argument('--topic', type=str, default='/planner/trajectory',
                       help='Topic to publish trajectory (default: /planner/trajectory)')

    known_args, unknown_args = parser.parse_known_args()
    rclpy.init(args=unknown_args)

    node = TrajectoryPublisher(known_args)

    print(f"\n{'='*50}")
    print(f"  TRAJECTORY PUBLISHER")
    print(f"{'='*50}")
    print(f"Publishing to topic: {node.topic}")
    print(f"Trajectory type: {node.trajectory_type}")
    print(f"Initial position: {node.initial_pos}")
    print(f"Trajectory start point: {node.trajectory_start_point}")
    print(f"Trajectory start velocity: {node.trajectory_start_velocity}")
    print(f"Radius: {node.radius} m")
    if node.trajectory_type == 'helix':
        print(f"Pitch: {node.pitch} m/revolution")
        distance_per_rev = np.sqrt((2 * np.pi * node.radius)**2 + node.pitch**2)
        time_per_rev = distance_per_rev / node.speed
        total_time = time_per_rev * node.n_revolutions
        final_altitude = node.initial_pos[2] + node.pitch * node.n_revolutions
        print(f"Distance per revolution: {distance_per_rev:.2f} m")
        print(f"Time per revolution: {time_per_rev:.2f} s")
        print(f"Total trajectory time: {total_time:.2f} s")
        print(f"Final altitude: {final_altitude:.2f} m")
    print(f"Revolutions: {node.n_revolutions}")
    print(f"Speed: {node.speed} m/s")
    print(f"Hover time: {node.hover_time} s")
    print(f"Transition time: {node.transition_time} s")
    print(f"Yaw: {node.yaw} rad")
    print(f"{'='*50}\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down trajectory publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
