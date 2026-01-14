# ROS2 Studio 🤖

A comprehensive ROS2 monitoring and management tool with GUI for performance monitoring, bag operations, and system diagnostics.

## Features

- 📊 **Performance Monitor** - Real-time CPU/memory monitoring for topics and nodes with graphical plots
- 🔴 **Bag Recorder** - Multi-topic recording with custom save locations
- ▶️ **Bag Player** - Playback with adjustable rate (0.1x-10x) and loop controls
- 🔄 **Bag to CSV Converter** - Full message deserialization to CSV format
- 🎛️ **System Dashboard** - System resources, ROS2 entities, network stats, and process monitoring

## Installation

```bash
# Install dependencies
sudo apt-get install python3-pyqt5 python3-matplotlib python3-psutil ros-$ROS_DISTRO-rosbag2-py

# Build
cd ~/ros2_ws
colcon build --packages-select ros2_studio
source install/setup.bash

# Launch
ros2 studio
```

## Usage

Launch the GUI and select features from the dropdown menu:
1. **Performance Monitor** - Select Topics/Nodes and view real-time metrics
2. **Bag Recorder** - Select topics, choose location, start/stop recording
3. **Bag Player** - Load bag, set rate/loop options, control playback
4. **CSV Converter** - Load bag, select topics, convert to CSV
5. **System Dashboard** - View system resources, ROS2 entities, network, and processes

## Project Structure

```
ros2_studio/
├── ros2_studio/
│   ├── core/          # Backend (monitoring, recording, playback, conversion, dashboard)
│   ├── gui/           # UI widgets for each feature
│   ├── command/       # ROS2 CLI extension
│   └── utils/         # Helper functions
├── package.xml        # Dependencies
└── setup.py           # Entry points
```

## Requirements

- ROS2 (Foxy/Galactic/Humble/later)
- Python 3.8+
- PyQt5, matplotlib, psutil, rosbag2_py

## Screenshots

### Performance Monitor
![Performance Monitor](ros2_studio/performanc%20metrics.png)
*Real-time CPU, memory, and frequency monitoring with graphical plots*

### Bag Recorder
![Bag Recorder](ros2_studio/bag%20recorder.png)
*Multi-topic selection with recording controls*

### Bag Player
![Bag Player](ros2_studio/bag%20play.png)
*Playback control with adjustable rate and loop options*

### Bag to CSV Converter
![CSV Converter](ros2_studio/convert%20to%20csv.png)
*Convert bag topics to CSV with full message deserialization*

### System Dashboard
![System Dashboard](ros2_studio/systm%20dashboard.png)
*System resources, ROS2 entities, network stats, and process monitoring*

## License

Apache License 2.0

## Author

Sourav Hawaldar (sourav.hawaldar@gmail.com)

## Repository

GitHub: [https://github.com/Sourav0607/ROS2-STUDIO](https://github.com/Sourav0607/ROS2-STUDIO)

---

**Note**: This is an independent community tool, not officially affiliated with Open Robotics or the ROS project.
