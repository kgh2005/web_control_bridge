# web_control_bridge

`web_control_bridge` is a ROS 2 package that communicates with **four robots via UDP**. It receives data from each robot, converts it into ROS 2 messages, and publishes it to corresponding topics. The system is designed to work in real-time with a web-based interface.

This package is used in conjunction with the web visualization frontend at:  
🔗 [https://github.com/kgh2005/robot_control_web.git](https://github.com/kgh2005/robot_control_web.git)  
That repository provides a web page that visualizes data received via UDP in real-time.

---

## 📡 Features

- Receives robot data via UDP
- Parses and publishes data to ROS 2 topics based on robot ID
- Supports interaction with 4 separate robots
- Integrates with web frontend using `rosbridge_websocket` and `roslibjs`

---

## 🌐 Web Visualization

- Web Interface Repository:  
  🔗 [https://github.com/kgh2005/robot_control_web.git](https://github.com/kgh2005/robot_control_web.git)

- This frontend connects to ROS 2 using `rosbridge_websocket` and displays real-time robot states including position, orientation, and ball location.

---

## ⚙️ Environment

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **rosbridge_server** (for web-ROS communication)
- **roslibjs** (JavaScript library for ROS)

---

## 🚀 How to Run

```bash
# Build the package inside your colcon workspace
colcon build --packages-select web_control_bridge

# Source the environment
source install/setup.bash

# Run the receiver node
ros2 launch web_control_bridge web_control_bridge_launch.py 