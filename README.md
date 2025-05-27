# 🏎️ Pure Pursuit ROS 2 Node

This is a **ROS 2 (Python)** implementation of the Pure Pursuit controller for Ackermann steering vehicles. It subscribes to a path and odometry topic and publishes steering and speed commands using `AckermannDriveStamped`. It also publishes visualization markers to RViz.

---

## 📦 Package Overview

This node provides the following functionality:

* Subscribes to a path of type `nav_msgs/Path`
* Subscribes to odometry data of type `nav_msgs/Odometry`
* Publishes drive commands to `ackermann_msgs/AckermannDriveStamped`
* Publishes visualization markers for the lookahead point and lookahead circle
* Responds to keyboard input for activating/deactivating autonomous velocity

---

## 🚀 Launching the Node

### Prerequisites

* ROS 2 Foxy, Galactic, Humble, or newer
* `ackermann_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `visualization_msgs` installed

### 1. **Add to Your Workspace**

Clone or copy the script into your ROS 2 workspace under a package directory (e.g., `pure_pursuit_pkg/nodes/pure_pursuit_node.py`).

Make sure it's executable:

```bash
chmod +x pure_pursuit_node.py
```

### 2. **Launch with Parameters**

Use `ros2 run` or include the node in a launch file. You must provide the necessary parameters.

#### Example CLI Launch:

```bash
ros2 run pure_pursuit_pkg pure_pursuit_node.py \
  --ros-args \
  -p max_lookahead_distance:=2.5 \
  -p min_lookahead_distance:=1.0 \
  -p max_velocity:=2.0 \
  -p min_velocity:=0.5 \
  -p kp:=1.5 \
  -p kd:=0.3 \
  -p is_antiClockwise:=false \
  -p k_sigmoid:=8.0 \
  -p cmd_vel_topic:=/ackermann_cmd \
  -p odometry_topic:=/odom \
  -p path_topic:=/pp_path
```

---

## 🧭 Runtime Behavior

* Press **`a`** on the keyboard to activate velocity control (autonomous driving starts).
* Release the `a` key to stop movement (speed goes to 0).
* Press **`ESC`** to stop the node’s listener thread.

---

## 📡 Topics

### Subscribed

| Topic      | Type                | Description           |
| ---------- | ------------------- | --------------------- |
| `/odom`    | `nav_msgs/Odometry` | Odometry input        |
| `/pp_path` | `nav_msgs/Path`     | Trajectory path input |

### Published

| Topic               | Type                                   | Description                                |
| ------------------- | -------------------------------------- | ------------------------------------------ |
| `/ackermann_cmd`    | `ackermann_msgs/AckermannDriveStamped` | Steering and speed output                  |
| `/lookahead_marker` | `visualization_msgs/Marker`            | Red sphere at the current lookahead point  |
| `/lookahead_circle` | `visualization_msgs/Marker`            | Green circle around vehicle for LAD radius |

---

## 🛠️ Parameters

| Name                     | Type   | Description                                        | Example Value    |
| ------------------------ | ------ | -------------------------------------------------- | ---------------- |
| `max_lookahead_distance` | double | Maximum lookahead distance                         | `2.5`            |
| `min_lookahead_distance` | double | Minimum lookahead distance                         | `1.0`            |
| `max_velocity`           | double | Maximum velocity for the vehicle                   | `2.0`            |
| `min_velocity`           | double | Minimum velocity for the vehicle                   | `0.5`            |
| `kp`                     | double | Proportional gain for steering control             | `1.5`            |
| `kd`                     | double | Derivative gain for steering control               | `0.3`            |
| `cmd_vel_topic`          | string | Topic to publish Ackermann drive commands          | `/ackermann_cmd` |
| `odometry_topic`         | string | Topic to receive odometry messages                 | `/odom`          |
| `path_topic`             | string | Topic to receive path messages                     | `/pp_path`       |
| `is_antiClockwise`       | bool   | Whether to reverse the path direction              | `false`          |
| `k_sigmoid`              | double | Steepness of sigmoid function for velocity control | `8.0`            |

---

## 📊 Visualizing in RViz

Add the following displays in RViz:

1. **Marker** for `/lookahead_marker` (red dot at lookahead point)
2. **Marker** for `/lookahead_circle` (green circular outline around robot)
3. **Path** for `/pp_path`
4. **Odometry** for `/odom`

Ensure `Fixed Frame` is set to `map`.

---

## 🧪 Testing Tips

* Provide a static or dynamically generated path on `/pp_path`.
* Use SLAM or simulation to feed odometry on `/odom`.
* If your path includes orientation and velocity (e.g., as the third field), the node will use them.

---

## 🧯 Emergency Stop

Letting go of the `a` key instantly sets speed to 0.

---

## 🧰 Debugging

* Logs will indicate current gamma and speed mode (`<opt_vel>` or `<sigmoidally>`).
* If "No lookahead point found" appears, check if the path is loaded and that odometry is valid.
* Make sure you are actively pressing `a` for the vehicle to move.

---

