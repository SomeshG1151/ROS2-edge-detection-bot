# ROS2 LiDAR-Based Edge Detection Bot 🤖📡

A modular ROS2-based mobile robot simulation using a simulated LiDAR sensor in Gazebo for environmental perception and edge/boundary detection.

This project demonstrates structured ROS2 architecture, sensor integration, ros2_control configuration, and simulation-based robotic perception.

---

# 📌 Project Overview

This project simulates a mobile robot in Gazebo equipped with a LiDAR sensor.  
The robot processes LaserScan data to detect edges, boundaries, or environmental discontinuities.

The system is divided into modular ROS2 packages for clean architecture and scalability.

---

# 🏗 System Architecture

```
ROS2 Workspace
│
├── bot_description   → URDF, LiDAR definition, simulation models
├── bot_controller    → ros2_control + controller configs
├── bot_bringup       → Launch system for simulation
└── bot_script        → LiDAR processing node
```

---

# 🛠 Tech Stack

- ROS2
- Gazebo
- RViz
- Python
- LiDAR (LaserScan)
- Xacro (URDF macros)
- ros2_control

---

# 📦 Package Details

## 1️⃣ bot_description

Contains:

- Robot URDF (`.xacro`)
- LiDAR sensor configuration
- Gazebo plugins
- World files
- RViz config

Key files:
- `urdf/bot.urdf.xacro`
- `urdf/bot_sensors.xacro`
- `launch/gazebo.launch.py`
- `worlds/empty.world`

The LiDAR sensor is defined in URDF and simulated using Gazebo plugins.

---

## 2️⃣ bot_controller

Handles:

- ros2_control configuration
- Controller YAML files
- Joint/state control integration

Key file:
- `config/bot_controllers.yaml`

---

## 3️⃣ bot_bringup

Responsible for:

- Launching full simulation
- Spawning robot in Gazebo
- Starting controllers
- Connecting sensor streams

Run using:

```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

---

## 4️⃣ bot_script

Implements:

- LiDAR data subscription (`sensor_msgs/msg/LaserScan`)
- Processing of scan ranges
- Edge/boundary detection logic
- Decision-making framework (expandable)

Main file:
- `edge_detection.py`

---

# 📡 LiDAR Processing Logic

The robot:

1. Subscribes to `/scan` topic
2. Reads LaserScan range data
3. Detects sudden range discontinuities
4. Identifies edges or boundaries
5. Publishes processed results (optional)

This demonstrates perception pipeline implementation using range-based sensing.

---

# 🚀 Installation & Setup

## 1️⃣ Clone the Repository

```bash
git clone https://github.com/SomeshG1151/ROS2-edge-detection-bot.git
cd ROS2-edge-detection-bot
```

---

## 2️⃣ Build Workspace

```bash
colcon build
```

---

## 3️⃣ Source Environment

```bash
source install/setup.bash
```

---

# ▶ Running the Simulation

Launch Gazebo with robot:

```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

Visualize in RViz:

```bash
ros2 launch bot_description display.launch.py
```

---

# 🧠 Learning Outcomes

This project demonstrates:

- Multi-package ROS2 workspace design
- LiDAR sensor integration in URDF
- Gazebo plugin configuration
- LaserScan data processing
- ros2_control usage
- Simulation-driven robotics development
- Modular robotics software design

---

# 🔮 Future Improvements

- Obstacle avoidance
- Autonomous navigation
- SLAM integration
- Mapping and localization
- PID tuning
- MoveIt integration
- Real robot hardware deployment

---

# 📸 Add Screenshots

(Add Gazebo + RViz screenshots here)

---

# 👤 Author

Somesh Gupta  
Robotics | ROS2 | Autonomous Systems | Simulation  

---

# 📜 License

MIT License
