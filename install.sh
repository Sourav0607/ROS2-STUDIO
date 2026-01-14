#!/bin/bash
# ROS2 Studio Installation Script

set -e

echo "========================================="
echo "ROS2 Studio Installation Script"
echo "========================================="

# Check if ROS2 is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 not detected. Please source your ROS2 installation first."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Detected ROS2 Distribution: $ROS_DISTRO"

# Check if workspace exists, create if not
if [ -z "$1" ]; then
    WORKSPACE="$HOME/ros2_ws"
else
    WORKSPACE="$1"
fi

echo "Using workspace: $WORKSPACE"

# Create workspace if it doesn't exist
if [ ! -d "$WORKSPACE/src" ]; then
    echo "Creating workspace at $WORKSPACE"
    mkdir -p "$WORKSPACE/src"
fi

# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python3-pyqt5 \
    python3-matplotlib \
    python3-psutil \
    ros-$ROS_DISTRO-rosbag2-py

# Clone repository
echo ""
echo "Cloning ROS2 Studio repository..."
cd "$WORKSPACE/src"

if [ -d "ros2_studio" ]; then
    echo "ros2_studio already exists, pulling latest changes..."
    cd ros2_studio
    git pull
    cd ..
else
    git clone https://github.com/Sourav0607/ROS2-STUDIO.git ros2_studio
fi

# Build package
echo ""
echo "Building ROS2 Studio..."
cd "$WORKSPACE"
colcon build --packages-select ros2_studio --symlink-install

# Source workspace
echo ""
echo "========================================="
echo "Installation Complete!"
echo "========================================="
echo ""
echo "To use ROS2 Studio, run:"
echo "  source $WORKSPACE/install/setup.bash"
echo "  ros2 studio"
echo ""
echo "Add this to your ~/.bashrc to make it permanent:"
echo "  echo 'source $WORKSPACE/install/setup.bash' >> ~/.bashrc"
echo ""
