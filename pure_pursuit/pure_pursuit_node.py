#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from pynput import keyboard

class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("max_lookahead_distance", 1.0)
        self.declare_parameter("min_lookahead_distance", 1.0)
        self.declare_parameter("max_velocity", 0.0)
        self.declare_parameter("min_velocity", 0.0)
        self.declare_parameter("cmd_vel_topic", "/d")
        self.declare_parameter("odometry_topic", "/o")
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("is_antiClockwise", False)
        self.declare_parameter("k_sigmoid", 8.0)
        self.declare_parameter("path_topic", "")

        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.min_velocity = self.get_parameter("min_velocity").get_parameter_value().double_value
        self.min_lad = self.get_parameter("min_lookahead_distance").get_parameter_value().double_value
        self.max_lad = self.get_parameter("max_lookahead_distance").get_parameter_value().double_value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odometry_topic").get_parameter_value().string_value
        self.is_antiClockwise = self.get_parameter("is_antiClockwise").get_parameter_value().bool_value
        self.k_sigmoid = self.get_parameter("k_sigmoid").get_parameter_value().double_value
        self.path_topic = self.get_parameter("path_topic").get_parameter_value().string_value

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(AckermannDriveStamped, self.cmd_vel_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_update_cb, 10)
        self.path = [] # a tuple of (x, y, v) 

        self.lookahead_marker_pub = self.create_publisher(Marker, "/lookahead_marker", 10)
        self.lookahead_circle_pub = self.create_publisher(Marker, "/lookahead_circle", 10)
        self.prev_gamma = 0.0
        self.activate_autonomous_vel = False 
        self.lookahead_distance = 0.0
        self.odometry = Odometry()
        
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

    def on_press(self, key):
        try:
            if key.char == 'a':
                self.activate_autonomous_vel = True 
        except AttributeError:
            self.get_logger().warn("error while sending.. :(")

    def on_release(self, key):
        # Stop the robot when the key is released
        # self.start_algorithm = False
        self.activate_autonomous_vel = False
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def path_update_cb(self, msg:Path):
        self.path.clear() # to clear the path
        for i in range(len(msg.poses)):
            self.path.append((msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.orientation.w))
        if self.is_antiClockwise:
            self.path.reverse()
        self.get_logger().info(f"path has been updated...")

    def odom_callback(self, msg:Odometry):
        self.odometry = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.publish_lookahead_circle(x, y)
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.lookahead_distance = self.get_lad_thresh(msg.twist.twist.linear.x)
        lookahead_point, closest_point = self.find_lookahead_point(x, y)
        if lookahead_point is None:
            self.get_logger().warn("No lookahead point found")
            return

        self.pursuit_the_point(lookahead_point, x, y, yaw, closest_point)
        self.publish_lookahead_marker(lookahead_point)

    def get_lad_thresh(self, v):
        # lad = m*v + c
        m = (self.max_lad - self.min_lad)/(self.max_velocity - self.min_velocity)
        c = self.max_lad - m * self.max_velocity
        lad = m * v + c
        if lad < self.min_lad:
            lad = self.min_lad
        if lad > self.max_lad:
            lad = self.max_lad
        return lad
    
    def pursuit_the_point(self, lookahead_point, x, y, yaw, closest_point):
        lx, ly = self.transform_to_vehicle_frame(lookahead_point, x, y, yaw)

        gamma = 2 * ly / (self.lookahead_distance ** 2)
        d_controller = (self.prev_gamma - gamma)*self.kd
        p_controller = self.kp*gamma
        self.prev_gamma = gamma
        steering_angle = p_controller + d_controller
        
        ackermann = AckermannDriveStamped()
        if self.activate_autonomous_vel:
            if closest_point[2] > 0.0: # that means the published path has velocity
                ackermann.drive.speed = closest_point[2] # the velocity at this instance
                self.get_logger().info(f'gamma: {gamma} vel: {ackermann.drive.speed} <opt_vel>')                
            else:
                ackermann.drive.speed = self.find_linear_vel_steering_controlled_sigmoidally(gamma)
                self.get_logger().info(f'gamma: {gamma} vel: {ackermann.drive.speed} <sigmoidally>')
        else:
            ackermann.drive.speed = 0.0
        ackermann.drive.steering_angle = steering_angle
        self.cmd_vel_pub.publish(ackermann)

    def find_lookahead_point(self, x, y):
        closest_idx = 0
        min_dist = float('inf')
        # First find the closest path point to the car
        for i, point in enumerate(self.path):
            dx = point[0] - x
            dy = point[1] - y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Now search only forward from that point
        for i in range(closest_idx, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= self.lookahead_distance:
                return self.path[i], self.path[closest_idx]

        # If no point was found, assume starting and reset closest idx
        closest_idx = 0
        for i in range(closest_idx, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= self.lookahead_distance:
                return self.path[i], self.path[closest_idx]

        return None, None

    def transform_to_vehicle_frame(self, point, x, y, yaw):
        dx = point[0] - x
        dy = point[1] - y
        transformed_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        transformed_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        return transformed_x, transformed_y

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_linear_vel_steering_controlled_rationally(self, gamma): # using the rational profile
        k = 7.0  # Increase for steeper drop
        vel = self.min_velocity + (self.max_velocity - self.min_velocity) / (1 + k * abs(gamma))
        return max(self.min_velocity, min(self.max_velocity, vel))

    def compute_c(self, v_min, v_max, k):
        return  -1*(1 / k) * np.log((v_max - v_max*0.999) / (v_max*0.999 - v_min))
    
    def find_linear_vel_steering_controlled_sigmoidally(self, gamma): # Sigmoid formula
        k = self.k_sigmoid # increase this variable if you want to make the curve more steep
        vel = self.min_velocity + ((self.max_velocity - self.min_velocity) / (1 + np.exp(k * (abs(gamma) - self.compute_c(v_min=self.min_velocity, v_max=self.max_velocity, k=k)))))
          # Clamp velocity to safety bounds
        vel = max(self.min_velocity, min(self.max_velocity, vel))
        return vel

    def find_linear_vel_steering_controlled(self, gamma):
        # vel = m*gamma + c
        self.min_gamma = 0.0
        self.max_gamma = 2/self.lookahead_distance
        m = (self.min_velocity - self.max_velocity)/(self.max_gamma - self.min_gamma)
        c = self.min_velocity - m*(self.max_gamma)
        vel = m*(gamma) + c
        if vel < self.min_velocity:
            vel = self.min_velocity
        if vel > self.max_velocity:
            vel = self.max_velocity
        return vel

    def publish_lookahead_marker(self, point):
        marker = Marker()
        marker.header.frame_id = 'map'  # or 'map' depending on your frame
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
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.lookahead_marker_pub.publish(marker)

    def publish_lookahead_circle(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'  # or 'map' if you're using that
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  # line thickness
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Create circle points
        resolution = 60  # more = smoother circle
        for i in range(resolution + 1):
            angle = 2 * math.pi * i / resolution
            px = x + self.lookahead_distance * math.cos(angle)
            py = y + self.lookahead_distance * math.sin(angle)
            p = Point()  # Dummy initialization to get geometry_msgs/Point
            p.x = px
            p.y = py
            p.z = 0.05
            marker.points.append(p)

        self.lookahead_circle_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()