#!/usr/bin/env python3
"""
CSV Racing Line Publisher Module

This module implements a ROS2 node that reads waypoints from a CSV file
and publishes them as a Path message for visualization and path following.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import csv
import os
from geometry_msgs.msg import Point
import numpy as np


class PathPublisher(Node):
    """
    A ROS2 node that publishes racing line paths from CSV files.

    This node reads waypoint data from a CSV file and publishes it as a ROS Path
    message for visualization in RViz and use by path following algorithms.
    """

    def __init__(self):
        """
        Initialize the PathPublisher node.

        Sets up parameters, loads the path from CSV, and publishes it.
        """
        super().__init__("csv_path_pub")

        # Declare parameters with default values
        self.declare_parameter("path_topic", "/csv_pp_path")
        self.declare_parameter(
            "csv_path",
            "/home/fam/Desktop/F1Tenth/Repos/trajectory_planning/path/cantine.csv",
        )
        self.declare_parameter("speed_factor", 1.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("inverse", False)

        # Get parameter values
        self.csv_path = (
            self.get_parameter("csv_path").get_parameter_value().string_value
        )
        self.path_topic = (
            self.get_parameter("path_topic").get_parameter_value().string_value
        )
        self.speed_factor = (
            self.get_parameter(
                "speed_factor").get_parameter_value().double_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.inverse = self.get_parameter(
            "inverse").get_parameter_value().bool_value

        # Create publisher and load/publish path
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.path = self.load_path_from_csv(self.csv_path)

        if self.inverse:
            self.path = self.path[::-1]

        for i in range(100):
            self.publish_path()

    def load_path_from_csv(self, csv_path):
        """
        Load path waypoints from a CSV file.

        Args:
            csv_path (str): Path to the CSV file containing waypoints

        Returns:
            list: List of tuples containing (x, y, velocity) for each waypoint
        """
        path = []
        try:
            with open(csv_path, newline="") as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    x, y, v = float(row[0]), float(row[1]), float(row[2])
                    path.append((x, y, v))
        except:
            self.get_logger().error("error while loading the path....")
        return path

    def publish_path(self):
        """
        Publish the loaded path as a ROS Path message.

        Creates a Path message with PoseStamped waypoints and publishes it.
        The velocity information is stored in the orientation.w field.
        """
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        for idx, point in enumerate(self.path):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = point[2] * self.speed_factor  # velocity
            path_msg.poses.append(pose)

        # self.get_logger().info("The path is loaded ..")
        self.path_publisher.publish(path_msg)


def main(args=None):
    """
    Main function to initialize and run the PathPublisher node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
