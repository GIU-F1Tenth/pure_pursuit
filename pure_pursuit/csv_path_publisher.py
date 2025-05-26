#!/usr/bin/env python3

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
    def __init__(self):
        super().__init__('csv_path_publisher_node')
        self.declare_parameter("path_topic")
        self.declare_parameter("csv_path")

        self.csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        self.path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.path = self.load_path_from_csv(self.csv_path)
        self.publish_path()

    def load_path_from_csv(self, csv_path):
        path = []
        with open(csv_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                x, y, v = float(row[0]), float(row[1]), float(row[2])
                path.append((x, y, v))
        return path
 
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for point in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = point[2] # velocity
            path_msg.poses.append(pose)
            self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()