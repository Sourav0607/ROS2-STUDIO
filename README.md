# ROS2 Studio 🤖

A comprehensive ROS2 monitoring and management tool with an intuitive GUI for performance monitoring, bag recording, bag playback, CSV conversion, and system diagnostics.

## Features

### 📊 Performance Monitor
- Monitor CPU and memory usage of ROS2 topics and nodes
- Real-time graphical display with X-Y axis plots
- Select topics or nodes from dropdown menus
- Track publishing frequency and connection statistics
- Historical data visualization (last 60 seconds)

### 🔴 Bag Recorder
- Select multiple topics to record from dropdown menu
- Choose custom save location
- Real-time recording duration display
- Automatic timestamp-based naming
- Start/stop recording with one click

### ▶️ Bag Player
- Browse and select recorded bag files
- View detailed bag information (topics, duration, size)
- Adjustable playback rate (0.1x to 10x)
- Loop playback option
- Pause/resume controls

### 🔄 Bag to CSV Converter
- Convert ROS2 bag files to CSV format
- Automatic topic detection from bag files
- Select specific topics to convert
- Full message deserialization with all fields
- Nested message field flattening

### 🎛️ System Dashboard
- Real-time system resource monitoring (CPU, Memory, Disk)
- Complete ROS2 entities overview (Topics, Nodes with statistics)
- Network interface monitoring and traffic graphs
- ROS-related process tracking
- Historical resource usage graphs

## Installation

### Prerequisites
- ROS2 (Foxy, Galactic, Humble, or later)
- Python 3.8+
- PyQt5
- psutil
- matplotlib

### System Dependencies

```bash
# Install PyQt5
sudo apt-get install python3-pyqt5

# Install psutil
sudo apt-get install python3-psutil

# Install matplotlib
sudo apt-get install python3-matplotlib

# Install rosbag2 Python API
sudo apt-get install ros-$ROS_DISTRO-rosbag2-py
```

### Build Instructions

1. Navigate to your ROS2 workspace:
```bash
cd ~/ros2_ws
```

2. Build the package:
```bash
colcon build --packages-select ros2_studio
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch ROS2 Studio

Simply run:
```bash
ros2 studio
```

This will open the ROS2 Studio GUI with all features accessible from the main window.

### Alternative Launch Method

You can also run directly:
```bash
ros2_studio_gui
```

## GUI Overview

### Main Window
The main window provides a feature selection dropdown with five options:
1. **📊 Performance Monitor** - Monitor topic and node performance
2. **🔴 Bag Recorder** - Record ROS2 bags
3. **▶️ Bag Player** - Play recorded bags
4. **🔄 Bag to CSV Converter** - Convert bags to CSV
5. **🎛️ System Dashboard** - System diagnostics and ROS2 overview

### Performance Monitor
1. Select monitor type (Topics or Nodes)
2. Choose an item from the dropdown
3. View real-time metrics and graphs
4. CPU, Memory, and Frequency displayed with live updates

### Bag Recorder
1. Click "Refresh Topics" to see available topics
2. Select topics you want to record (multi-select supported)
3. Choose save location or use default `~/ros2_bags`
4. Click "Start Recording"
5. Click "Stop Recording" when done

### Bag Player
1. Browse for a bag file or directory
2. Click "Load Bag Info" to view bag details
3. Set playback rate and loop options
4. Click "Play" to start playback
5. Use Pause/Resume/Stop controls as needed

### Bag to CSV Converter
1. Browse and select a bag folder
2. Click "Load Topics" to see available topics
3. Select topics you want to convert
4. Choose output directory
5. Click "Convert to CSV"
6. CSV files created with full message data

### System Dashboard
1. View real-time system metrics in overview cards
2. Navigate between tabs:
   - **System Resources**: CPU/Memory/Disk usage with graphs
   - **ROS2 Entities**: Complete topics and nodes tables
   - **Network**: Interface stats and traffic monitoring
   - **Processes**: Top ROS-related processes by CPU
3. Click "Refresh" to update all data

## Project Structure

```
ros2_studio/
├── package.xml                 # Package metadata and dependencies
├── setup.py                    # Python package setup with entry points
├── setup.cfg                   # Package configuration
├── resource/
│   └── ros2_studio            # Package marker
├── ros2_studio/
│   ├── __init__.py
│   ├── main.py                # Application entry point
│   ├── command/               # ROS2 CLI extension
│   │   ├── __init__.py
│   │   └── studio.py          # Studio command implementation
│   ├── core/                  # Backend logic
│   │   ├── __init__.py
│   │   ├── performance_monitor.py  # Performance monitoring
│   │   ├── bag_recorder.py         # Bag recording
│   │   ├── bag_player.py           # Bag playback
│   │   ├── bag_converter.py        # Bag to CSV conversion
│   │   └── system_dashboard.py     # System diagnostics
│   ├── gui/                   # GUI components
│   │   ├── __init__.py
│   │   ├── main_window.py          # Main application window
│   │   ├── monitoring_widget.py    # Performance monitor UI
│   │   ├── bag_record_widget.py    # Bag recorder UI
│   │   ├── bag_play_widget.py      # Bag player UI
│   │   ├── bag_convert_widget.py   # CSV converter UI
│   │   └── system_dashboard_widget.py  # Dashboard UI
│   └── utils/                 # Utility functions
│       ├── __init__.py
│       └── ros2_utils.py           # ROS2 helper functions
└── test/                      # Test files
```

## Configuration

### Default Settings

- **Bag Save Location**: `~/ros2_bags`
- **Performance Update Rate**: 1 second
- **Graph History**: 60 seconds
- **Default Playback Rate**: 1.0x

### Customization

You can modify default settings by editing the respective widget files in `ros2_studio/gui/`.

## Troubleshooting

### Issue: "No topics available"
**Solution**: Make sure you have ROS2 nodes running that publish topics.

```bash
# In another terminal, run a demo talker
ros2 run demo_nodes_cpp talker
```

### Issue: GUI doesn't open
**Solution**: Ensure PyQt5 is properly installed:
```bash
pip3 install PyQt5
# or
sudo apt-get install python3-pyqt5
```

### Issue: Permission denied when recording
**Solution**: Ensure the save directory has write permissions:
```bash
mkdir -p ~/ros2_bags
chmod 755 ~/ros2_bags
```

### Issue: "ros2 studio" command not found
**Solution**: Make sure you've sourced your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Development

### Adding New Features

1. Add backend logic in `ros2_studio/core/`
2. Create GUI widget in `ros2_studio/gui/`
3. Integrate widget in `main_window.py`
4. Update this README

### Running Tests

```bash
cd ~/ros2_ws
colcon test --packages-select ros2_studio
```

## Dependencies

- **rclpy**: ROS2 Python client library
- **ros2cli**: ROS2 command line interface
- **ros2topic**: Topic introspection tools
- **ros2node**: Node introspection tools
- **ros2bag**: Bag recording and playback
- **rosbag2_py**: Python API for bag operations
- **PyQt5**: GUI framework
- **psutil**: System and process utilities
- **matplotlib**: Plotting library

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

Apache License 2.0

## Author

Sourav (sourav.hawaldar@gmail.com)

## Version History

- **v0.1.0** (2026-01-14)
  - Initial release
  - Performance monitoring for topics and nodes
  - Bag recording with multi-topic selection
  - Bag playback with rate and loop controls
  - Bag to CSV converter with full message deserialization
  - System dashboard with diagnostics and ROS2 overview
  - Custom `ros2 studio` command
  - Custom robot icon branding

## Screenshots

### Main Window
The main window with feature selection dropdown.

### Performance Monitor
Real-time monitoring with CPU, memory, and frequency graphs.

### Bag Recorder
Multi-topic selection with recording controls.

### Bag Player
Bag playback with rate and loop options.

## Future Enhancements

- [ ] Add more detailed topic statistics (bandwidth, latency)
- [ ] Support for filtering topics by pattern
- [ ] Export performance metrics to CSV
- [ ] Add recording profiles for quick setup
- [ ] Support for multiple simultaneous recordings
- [ ] Enhanced bag analysis tools
- [ ] Dark theme support
- [ ] Configuration file support
- [ ] Image topic visualization in converter
- [ ] Custom message type support
- [ ] Multi-bag comparison tools

## Repository

GitHub: [https://github.com/Sourav0607/ROS2-STUDIO](https://github.com/Sourav0607/ROS2-STUDIO)

## Support

For issues, questions, or suggestions, please open an issue on the project repository.

---

**Enjoy using ROS2 Studio! 🎉**
