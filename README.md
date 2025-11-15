# Pure Pursuit Controller for F1TENTH Racing

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg)
![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)

A professional ROS2 implementation of the Pure Pursuit path following algorithm, specifically designed for F1TENTH autonomous racing. This controller provides adaptive lookahead distance, PID-based steering control, and sigmoid-based velocity control for optimal racing performance.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Topics](#topics)
- [Launch Files](#launch-files)
- [Visualization](#visualization)
- [Testing](#testing)
- [Contributing](#contributing)
- [License](#license)
- [Maintainers](#maintainers)

## Features

### Core Functionality
- **Pure Pursuit Algorithm**: Implements the classic pure pursuit path following algorithm
- **Adaptive Lookahead**: Dynamic lookahead distance based on current vehicle velocity
- **PID Steering Control**: Configurable proportional and derivative gains for precise steering
- **Sigmoid Velocity Control**: Smooth velocity transitions based on path curvature
- **Path Following**: Subscribes to Path messages for real-time path updates
- **Emergency Controls**: Safety features including emergency stop and pause functionality

### Advanced Features
- **Anti-skidding Control**: Velocity smoothing to prevent wheel skidding
- **Real-time Parameter Tuning**: Joystick-based parameter adjustment during operation
- **Visualization Support**: RViz markers for debugging and monitoring
- **Transform Integration**: Uses TF2 for robust coordinate frame transformations
- **Robust Error Handling**: Comprehensive error checking and recovery mechanisms

## Installation

### Prerequisites
- ROS2 (Humble recommended)
- Python 3.8+
- Required ROS2 packages:
  - `rclpy`
  - `ackermann_msgs`
  - `nav_msgs`
  - `geometry_msgs`
  - `std_msgs`
  - `sensor_msgs`
  - `visualization_msgs`
  - `tf2_ros`

### Build Instructions

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/your_ws/src
git clone <repository-url>
```

2. Build the package:
```bash
cd ~/your_ws
colcon build --packages-select pure_pursuit
source install/setup.bash
```

## Usage

### Quick Start

#### Launch Pure Pursuit Controller
```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```

### Manual Launch with Custom Parameters
```bash
ros2 run pure_pursuit pure_pursuit_node --ros-args --params-file /path/to/custom_params.yaml
```

### Joystick Controls
During operation, you can tune parameters in real-time using a joystick:
- **Y Button**: Increase derivative gain (kd)
- **A Button**: Decrease derivative gain (kd)
- **B Button**: Increase proportional gain (kp)
- **X Button**: Decrease proportional gain (kp)
- **LB Button**: Enable/disable autonomous velocity control

## Configuration

### Main Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_lookahead_distance` | double | 2.0 | Maximum lookahead distance (m) |
| `min_lookahead_distance` | double | 0.66 | Minimum lookahead distance (m) |
| `max_velocity` | double | 4.7 | Maximum vehicle velocity (m/s) |
| `min_velocity` | double | 2.2 | Minimum vehicle velocity (m/s) |
| `kp` | double | 0.45 | Proportional steering gain |
| `kd` | double | 1.2 | Derivative steering gain |
| `k_sigmoid` | double | 8.0 | Sigmoid steepness for velocity control |
| `vel_division_factor` | double | 1.0 | Velocity reduction factor |
| `skidding_velocity_thresh` | double | 0.3 | Anti-skidding threshold |
| `control_frequency` | double | 200.0 | Control loop frequency (Hz) |

### Topic Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cmd_vel_topic` | string | "/drive" | Ackermann command output |
| `odometry_topic` | string | "/odom" | Odometry input |
| `path_topic` | string | "/pp_path" | Path input topic |
| `joy_topic` | string | "/joy" | Joystick input |
| `pause_topic` | string | "/pause" | Emergency pause control |

### Transform Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tf_target` | string | "map" | Target coordinate frame |
| `tf_source` | string | "base_link" | Source coordinate frame |
| `tf_timeout` | double | 0.5 | Transform timeout (seconds) |

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Vehicle odometry data |
| `/pp_path` | `nav_msgs/Path` | Path to follow |
| `/pause` | `std_msgs/Bool` | Emergency pause control |
| `/joy` | `sensor_msgs/Joy` | Joystick input |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Vehicle control commands |
| `/lookahead_marker` | `visualization_msgs/Marker` | Lookahead point visualization |
| `/lookahead_circle` | `visualization_msgs/Marker` | Lookahead circle visualization |

## Launch Files

### `pure_pursuit.launch.py`
Main launch file that starts the pure pursuit controller with parameters loaded from `config/params.yaml`.

## Visualization

The controller provides comprehensive RViz visualization:

1. **Lookahead Point**: Red sphere marker showing the current target point
2. **Lookahead Circle**: Green circle showing the adaptive lookahead radius

### RViz Setup
Add these displays in RViz:
- Marker display for `/lookahead_marker`
- Marker display for `/lookahead_circle`  
- Odometry display for `/odom`

Set the fixed frame to `map`.

## Testing

### Simulation Testing
1. Launch your F1TENTH simulator
2. Start the pure pursuit controller:
```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```
3. Publish a path to `/pp_path` topic
4. Enable autonomous mode via joystick or topic

### Real Vehicle Testing
1. Ensure all sensors are calibrated
2. Start the controller with appropriate parameters
3. Publish a path to the `/pp_path` topic
4. Gradually tune parameters using joystick controls

### Debugging Tips
- Monitor console output for parameter changes and warnings
- Use RViz visualization to verify lookahead calculations
- Check TF transforms are available between `map` and `base_link`
- Verify path messages contain proper pose and velocity information

## Contributing

We welcome contributions! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch
3. Add comprehensive docstrings and comments
4. Test your changes thoroughly
5. Submit a pull request with detailed description

### Code Style
- Follow PEP 8 Python style guidelines
- Add type hints where appropriate
- Include comprehensive docstrings for all methods
- Use meaningful variable and function names

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Maintainers

This repository is actively maintained by:

- **Fam Shihata** - [fam@awadlouis.com](mailto:fam@awadlouis.com)
- **George Halim** - [georgehany064@gmail.com](mailto:georgehany064@gmail.com)

### Support

For questions, issues, or contributions:
- Open an issue on GitHub
- Contact the maintainers directly
- Check the documentation and examples

---

**Note**: This Pure Pursuit controller was developed for F1TENTH autonomous racing as part of research and educational activities at GIU Berlin. It has been tested in both simulation and real-world racing scenarios.
