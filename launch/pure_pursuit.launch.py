from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    param_path = os.path.join(get_package_share_directory("pure_pursuit"), "config", "params.yaml")
    pure_pursuit = Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[param_path],
            output='screen'
        )
    
    ld.add_action(pure_pursuit)

    return ld