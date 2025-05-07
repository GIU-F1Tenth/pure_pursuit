from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=['/home/ubuntu/giu_f1tenth_ws/software/src/planning/pure_pursuit/config/params.yaml'],
            output='screen'
        )
    ])
