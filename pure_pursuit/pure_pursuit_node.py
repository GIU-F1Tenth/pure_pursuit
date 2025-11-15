#!/usr/bin/env python3
"""
Pure Pursuit Controller for F1TENTH Autonomous Racing

This module implements a pure pursuit path following algorithm for autonomous
vehicle navigation in F1TENTH racing scenarios. The controller tracks a
predefined path while maintaining smooth steering and velocity control.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Bool
import math
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
import numpy as np
from tf2_ros import Buffer, TransformListener


def euler_from_quaternion(quaternion):
    """
    Convert quaternion to Euler angles.

    Converts quaternion (w in last place) to euler roll, pitch, yaw.
    This should be replaced when porting for ROS 2 Python tf_conversions is done.

    Args:
        quaternion (list): Quaternion as [x, y, z, w]

    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PurePursuit(Node):
    """
    Pure Pursuit Path Following Controller for F1TENTH Racing.

    This class implements a pure pursuit algorithm for autonomous vehicle navigation.
    It subscribes to odometry data and publishes Ackermann drive commands to follow
    a predefined path. The controller supports both CSV-based racing paths.

    Key Features:
        - Adaptive lookahead distance based on vehicle velocity
        - PID-based steering control with configurable gains
        - Sigmoid-based velocity control for smooth cornering
        - Support for both racing and solo driving modes
        - Real-time path switching capability
        - Joystick control for parameter tuning
        - Visualization markers for debugging

    Attributes:
        path (list): Current path as list of (x, y, velocity) tuples
        lookahead_distance (float): Current lookahead distance in meters
        is_active (bool): Controller activation state
        stop (bool): Emergency stop flag
    """

    def __init__(self):
        """
        Initialize the Pure Pursuit controller node.

        Sets up all ROS2 parameters, subscribers, publishers, and loads the initial path.
        """
        super().__init__("pure_pursuit_node")

        # Declare all parameters with default values
        self.declare_parameter("max_lookahead_distance", 1.0)
        self.declare_parameter("min_lookahead_distance", 1.0)
        self.declare_parameter("max_velocity", 0.0)
        self.declare_parameter("min_velocity", 0.0)
        self.declare_parameter("cmd_vel_topic", "/ackermann_cmd")
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("k_sigmoid", 8.0)
        self.declare_parameter("skidding_velocity_thresh", 0.0)
        self.declare_parameter("vel_division_factor", 1.0)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("pause_topic", "/pause")
        self.declare_parameter("control_frequency", 200.0)
        self.declare_parameter("tf_timeout", 0.5)
        self.declare_parameter("marker_resolution", 60)
        self.declare_parameter("subscriber_queue_size", 10)
        self.declare_parameter("publisher_queue_size", 10)
        self.declare_parameter("tf_target", "map")
        self.declare_parameter("tf_source", "base_link")

        # Load parameters
        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.max_velocity = self.get_parameter(
            "max_velocity").get_parameter_value().double_value
        self.min_velocity = self.get_parameter(
            "min_velocity").get_parameter_value().double_value
        self.min_lad = self.get_parameter(
            "min_lookahead_distance").get_parameter_value().double_value
        self.max_lad = self.get_parameter(
            "max_lookahead_distance").get_parameter_value().double_value
        self.cmd_vel_topic = self.get_parameter(
            "cmd_vel_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter(
            "odometry_topic").get_parameter_value().string_value
        self.path_topic = self.get_parameter(
            "path_topic").get_parameter_value().string_value
        self.k_sigmoid = self.get_parameter(
            "k_sigmoid").get_parameter_value().double_value
        self.skidding_velocity_thresh = self.get_parameter(
            "skidding_velocity_thresh").get_parameter_value().double_value
        self.vel_division_factor = self.get_parameter(
            "vel_division_factor").get_parameter_value().double_value
        self.joy_topic = self.get_parameter(
            "joy_topic").get_parameter_value().string_value
        self.pause_topic = self.get_parameter(
            "pause_topic").get_parameter_value().string_value
        self.control_frequency = self.get_parameter(
            "control_frequency").get_parameter_value().double_value
        self.tf_timeout = self.get_parameter(
            "tf_timeout").get_parameter_value().double_value
        self.marker_resolution = int(self.get_parameter(
            "marker_resolution").get_parameter_value().integer_value)
        self.queue_size = int(self.get_parameter(
            "subscriber_queue_size").get_parameter_value().integer_value)
        self.pub_queue_size = int(self.get_parameter(
            "publisher_queue_size").get_parameter_value().integer_value)
        self.tf_target = self.get_parameter(
            "tf_target").get_parameter_value().string_value
        self.tf_source = self.get_parameter(
            "tf_source").get_parameter_value().string_value

        # Initialize subscribers and publishers
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, self.queue_size)
        self.cmd_vel_pub = self.create_publisher(
            AckermannDriveStamped, self.cmd_vel_topic, self.pub_queue_size)
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_update_cb, self.queue_size)
        self.pause_sub = self.create_subscription(
            Bool, self.pause_topic, self.toggle_stop_cb, self.queue_size)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, self.queue_size)
        # Initialize TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create control timer with configurable frequency
        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.get_pose)

        # Initialize path data structures
        self.csv_race_path = []
        self.astar_path = []

        # Control state variables
        self.stop = False
        self.activate_autonomous_vel = True
        self.prev_gamma = 0.0
        self.lookahead_distance = 0.0
        self.odometry = Odometry()

        # Set initial path
        self.path = []

        # Initialize visualization publishers
        self.lookahead_marker_pub = self.create_publisher(
            Marker, "/lookahead_marker", self.pub_queue_size)
        self.lookahead_circle_pub = self.create_publisher(
            Marker, "/lookahead_circle", self.pub_queue_size)

        self.get_logger().info("Pure Pursuit Node initialized successfully")
        self.get_logger().info(
            f"Control frequency: {self.control_frequency} Hz")

    def toggle_stop_cb(self, msg: Bool):
        """
        Emergency stop callback to pause/resume the controller.

        Args:
            msg (Bool): True to stop, False to resume
        """
        if msg.data:
            self.stop = True
            self.get_logger().info("Pure Pursuit stopped")
        else:
            self.stop = False
            self.get_logger().info("Pure Pursuit resumed")

    def joy_callback(self, msg: Joy):
        """
        Joystick callback for real-time parameter tuning during operation.

        Button mapping:
        - Y (button 3): Increase kd by 0.1
        - A (button 1): Decrease kd by 0.1  
        - B (button 2): Increase kp by 0.1
        - X (button 0): Decrease kp by 0.1
        - LB (button 4): Enable autonomous velocity control

        Args:
            msg (Joy): Joystick message containing button and axis states
        """
        # Y button - Increase derivative gain
        if msg.buttons[3] == 1:
            self.kd += 0.1
            self.get_logger().info(f"kd increased to {self.kd:.2f}")

        # A button - Decrease derivative gain
        if msg.buttons[1] == 1:
            self.kd -= 0.1
            self.get_logger().info(f"kd decreased to {self.kd:.2f}")

        # B button - Increase proportional gain
        if msg.buttons[2] == 1:
            self.kp += 0.1
            self.get_logger().info(f"kp increased to {self.kp:.2f}")

        # X button - Decrease proportional gain
        if msg.buttons[0] == 1:
            self.kp -= 0.1
            self.get_logger().info(f"kp decreased to {self.kp:.2f}")

        # LB button - Toggle autonomous velocity control
        if msg.buttons[4] == 1:
            self.activate_autonomous_vel = True
        else:
            self.activate_autonomous_vel = False

    def path_update_cb(self, msg: Path):
        """
        Callback for switching between different path sources.

        Args:
            msg (String): Path source identifier ("astar_path" or "csv_race_path")
        """
        self.path = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            v = pose.pose.orientation.w  # Velocity stored in orientation.w
            self.path.append((x, y, v))

        self.get_logger().info(f"Path updated with {len(self.path)} points")

    def get_pose(self):
        """
        Main control loop that gets robot pose and executes pure pursuit control.

        This method is called at the configured control frequency. It:
        1. Gets the current robot pose from TF2
        2. Calculates lookahead distance based on current velocity
        3. Finds the appropriate lookahead point on the path
        4. Executes pure pursuit control to track that point
        5. Publishes visualization markers for debugging
        """
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.tf_target,          # target_frame
                self.tf_source,    # source_frame
                now,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )

            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convert quaternion to yaw angle
            orientation_list = [rot.x, rot.y, rot.z, rot.w]
            _, _, yaw = euler_from_quaternion(orientation_list)

            x, y = trans.x, trans.y

            # Publish visualization markers
            self.publish_lookahead_circle(x, y)

            # Calculate adaptive lookahead distance
            self.lookahead_distance = self.get_lad_thresh(
                self.odometry.twist.twist.linear.x)

            # Find the lookahead point on the path
            lookahead_point, closest_point, lookahead_index = self.find_lookahead_point(
                x, y)

            if lookahead_point is None:
                self.get_logger().warn("No lookahead point found")
                return

            # Execute pure pursuit control
            self.pursuit_the_point(
                lookahead_point, lookahead_index, x, y, yaw, closest_point)

            # Publish lookahead point marker for visualization
            self.publish_lookahead_marker(lookahead_point)

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

    def perp_distance_car_frame_lookahead_point(self, lookahead, x, y, yaw):
        """
        Calculate perpendicular distance from vehicle to lookahead point in vehicle frame.

        Args:
            lookahead (tuple): (x, y, v) coordinates of lookahead point
            x (float): Current vehicle x position
            y (float): Current vehicle y position  
            yaw (float): Current vehicle yaw angle

        Returns:
            float: Perpendicular distance (negative = left, positive = right)
        """
        lookahead_in_car_frame = self.transform_to_vehicle_frame(
            lookahead, x, y, yaw)
        # Negative because positive y is to the left
        return -lookahead_in_car_frame[1]

    def smooth_vel(self, curr_vel, target_vel) -> float:
        """
        Apply velocity smoothing to prevent wheel skidding and abrupt changes.

        Args:
            curr_vel (float): Current vehicle velocity
            target_vel (float): Desired target velocity

        Returns:
            float: Smoothed velocity command
        """
        vel = target_vel
        if (target_vel - curr_vel) > self.skidding_velocity_thresh:
            vel = self.skidding_velocity_thresh + curr_vel

        return vel

    def odom_callback(self, msg: Odometry):
        """
        Odometry callback to store the latest vehicle state.

        Args:
            msg (Odometry): Vehicle odometry containing pose and twist information
        """
        self.odometry = msg

    def get_lad_thresh(self, v):
        """
        Calculate adaptive lookahead distance based on current velocity.

        Uses linear interpolation between min/max lookahead distances based on
        the velocity range. Higher velocities get larger lookahead distances.

        Args:
            v (float): Current vehicle velocity in m/s

        Returns:
            float: Calculated lookahead distance in meters
        """
        # Linear interpolation: lad = m*v + c
        m = (self.max_lad - self.min_lad) / \
            (self.max_velocity - self.min_velocity)
        c = self.max_lad - m * self.max_velocity
        lad = m * v + c

        # Clamp to bounds
        lad = max(self.min_lad, min(self.max_lad, lad))
        return lad

    def pursuit_the_point(self, lookahead_point, lookahead_index, x, y, yaw, closest_point):
        """
        Execute pure pursuit control to track the lookahead point.

        This is the core control algorithm that:
        1. Transforms lookahead point to vehicle frame
        2. Calculates curvature (gamma) for steering control
        3. Applies PD control for steering angle
        4. Determines appropriate velocity based on path curvature
        5. Publishes Ackermann drive command

        Args:
            lookahead_point (tuple): (x, y, v) target point coordinates
            lookahead_index (int): Index of lookahead point in path
            x (float): Current vehicle x position
            y (float): Current vehicle y position
            yaw (float): Current vehicle yaw angle
            closest_point (tuple): (x, y, v) closest point on path to vehicle
        """
        # Transform lookahead point to vehicle frame
        lx, ly = self.transform_to_vehicle_frame(lookahead_point, x, y, yaw)

        # Calculate curvature (gamma) for pure pursuit steering
        gamma = 2 * ly / (self.lookahead_distance ** 2)

        # PD control for steering angle
        d_controller = (self.prev_gamma - gamma) * self.kd
        p_controller = self.kp * gamma
        self.prev_gamma = gamma
        steering_angle = p_controller + d_controller

        # Create Ackermann drive command
        ackermann = AckermannDriveStamped()
        ackermann.header.stamp = self.get_clock().now().to_msg()
        ackermann.header.frame_id = 'base_link'

        # Determine velocity based on autonomous mode and path information
        if self.activate_autonomous_vel and not self.stop:
            if closest_point[2] > 0.0:  # Path has velocity information
                ackermann.drive.speed = closest_point[2] / \
                    self.vel_division_factor
            else:  # Use sigmoid velocity control based on steering curvature
                ackermann.drive.speed = self.find_linear_vel_steering_controlled_sigmoidally(
                    gamma) / self.vel_division_factor

        # Check if perpendicular distance is too large (off-track detection)
        perp_distance = self.perp_distance_car_frame_lookahead_point(
            lookahead_point, x, y, yaw)
        if perp_distance >= self.lookahead_distance:
            lookahead_index += 8    # Skip ahead in path
            ackermann.drive.speed /= 2.0  # Reduce speed for safety
            if lookahead_index < len(self.path):
                lookahead_point = self.path[lookahead_index]

        # Apply velocity smoothing to prevent skidding
        ackermann.drive.speed = self.smooth_vel(
            self.odometry.twist.twist.linear.x, ackermann.drive.speed)
        ackermann.drive.steering_angle = steering_angle

        # Publish the control command
        self.cmd_vel_pub.publish(ackermann)

    def find_lookahead_point(self, x, y):
        """
        Find the appropriate lookahead point on the path for pure pursuit control.

        This method:
        1. Finds the closest point on the path to the vehicle
        2. Searches forward from that point for a point at lookahead distance
        3. Returns the lookahead point, closest point, and lookahead index

        Args:
            x (float): Current vehicle x position
            y (float): Current vehicle y position

        Returns:
            tuple: (lookahead_point, closest_point, lookahead_index) or (None, None, None)
                  if no suitable point is found
        """
        closest_idx = 0
        min_dist = float('inf')

        # Find the closest path point to the vehicle
        for i, point in enumerate(self.path):
            dx = point[0] - x
            dy = point[1] - y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Search forward from closest point for lookahead point
        for i in range(closest_idx, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= self.lookahead_distance:
                return self.path[i], self.path[closest_idx], i

        # If no point found, search from beginning (path wrap-around)
        for i in range(0, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= self.lookahead_distance:
                return self.path[i], self.path[closest_idx], i

        return None, None, None

    def transform_to_vehicle_frame(self, point, x, y, yaw):
        """
        Transform a point from global frame to vehicle frame.

        Args:
            point (tuple): (x, y, v) point coordinates in global frame
            x (float): Vehicle x position in global frame
            y (float): Vehicle y position in global frame
            yaw (float): Vehicle yaw angle

        Returns:
            tuple: (x, y) coordinates in vehicle frame
        """
        dx = point[0] - x
        dy = point[1] - y
        transformed_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        transformed_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        return transformed_x, transformed_y

    def get_yaw_from_quaternion(self, q):
        """
        Extract yaw angle from quaternion orientation.

        Args:
            q: Quaternion object with w, x, y, z components

        Returns:
            float: Yaw angle in radians
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_linear_vel_steering_controlled_rationally(self, gamma):
        """
        Calculate velocity using rational function based on steering curvature.

        Args:
            gamma (float): Curvature value for current steering

        Returns:
            float: Calculated velocity within min/max bounds
        """
        k = 7.0  # Steepness parameter
        vel = self.min_velocity + \
            (self.max_velocity - self.min_velocity) / (1 + k * abs(gamma))
        return max(self.min_velocity, min(self.max_velocity, vel))

    def compute_c(self, v_min, v_max, k):
        """
        Compute the sigmoid offset parameter for velocity control.

        Args:
            v_min (float): Minimum velocity
            v_max (float): Maximum velocity  
            k (float): Sigmoid steepness parameter

        Returns:
            float: Sigmoid offset parameter
        """
        return -1 * (1 / k) * np.log((v_max - v_max * 0.999) / (v_max * 0.999 - v_min))

    def find_linear_vel_steering_controlled_sigmoidally(self, gamma):
        """
        Calculate velocity using sigmoid function based on steering curvature.

        This provides smooth velocity transitions that slow down in tight corners
        and speed up on straights, improving both safety and performance.

        Args:
            gamma (float): Curvature value for current steering

        Returns:
            float: Calculated velocity within min/max bounds
        """
        k = self.k_sigmoid
        c = self.compute_c(v_min=self.min_velocity,
                           v_max=self.max_velocity, k=k)
        vel = self.min_velocity + ((self.max_velocity - self.min_velocity) /
                                   (1 + np.exp(k * (abs(gamma) - c))))

        # Clamp velocity to safety bounds
        vel = max(self.min_velocity, min(self.max_velocity, vel))
        return vel

    def find_linear_vel_steering_controlled(self, gamma):
        """
        Calculate velocity using linear relationship with steering curvature.

        Args:
            gamma (float): Curvature value for current steering

        Returns:
            float: Calculated velocity within min/max bounds
        """
        # Linear relationship: vel = m*gamma + c
        self.min_gamma = 0.0
        self.max_gamma = 2 / self.lookahead_distance
        m = (self.min_velocity - self.max_velocity) / \
            (self.max_gamma - self.min_gamma)
        c = self.min_velocity - m * self.max_gamma
        vel = m * gamma + c

        # Clamp to bounds
        vel = max(self.min_velocity, min(self.max_velocity, vel))
        return vel

    def publish_lookahead_marker(self, point):
        """
        Publish a visual marker for the current lookahead point.

        Args:
            point (tuple): (x, y, v) coordinates of the lookahead point
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.lookahead_marker_pub.publish(marker)

    def publish_lookahead_circle(self, x, y):
        """
        Publish a visual circle marker showing the current lookahead distance.

        Args:
            x (float): Vehicle x position (center of circle)
            y (float): Vehicle y position (center of circle)
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  # Line thickness
        marker.color.a = 1.0   # Fully opaque
        marker.color.r = 0.0
        marker.color.g = 1.0   # Green
        marker.color.b = 0.0

        # Create circle points around vehicle position
        for i in range(self.marker_resolution + 1):
            angle = 2 * math.pi * i / self.marker_resolution
            px = x + self.lookahead_distance * math.cos(angle)
            py = y + self.lookahead_distance * math.sin(angle)
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.05
            marker.points.append(p)

        self.lookahead_circle_pub.publish(marker)


def main(args=None):
    """
    Main function to initialize and run the Pure Pursuit node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Pure Pursuit node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
