"""
Launch file for Pure Pursuit Controller - Solo Mode

This launch file starts the pure pursuit controller with solo racing configuration.
Optimized for time trial racing with aggressive parameters.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for solo mode pure pursuit."""
    ld = LaunchDescription()

    # Get the package share directory
    package_share_dir = get_package_share_directory("pure_pursuit")

    # Path to the solo configuration file
    param_path = os.path.join(package_share_dir, "config", "params_solo.yaml")

    # Pure pursuit node with solo parameters
    pure_pursuit_solo = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[param_path],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(pure_pursuit_solo)

    return ld
