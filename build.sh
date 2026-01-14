#!/bin/bash

# ROS2 Studio Build Script
# This script builds and installs the ros2_studio package

set -e  # Exit on error

echo "=========================================="
echo "  ROS2 Studio Build Script"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Workspace: $WORKSPACE_DIR"
echo ""

# Check if we're in a ROS2 workspace
if [ ! -f "$WORKSPACE_DIR/src/ros2_studio/package.xml" ]; then
    echo -e "${RED}Error: Cannot find ros2_studio package!${NC}"
    echo "Make sure you're running this from the correct location."
    exit 1
fi

# Check for ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 not sourced. Attempting to source...${NC}"
    
    # Try common ROS2 installations
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ Sourced ROS2 Humble${NC}"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo -e "${GREEN}✓ Sourced ROS2 Iron${NC}"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        echo -e "${GREEN}✓ Sourced ROS2 Jazzy${NC}"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo -e "${GREEN}✓ Sourced ROS2 Foxy${NC}"
    else
        echo -e "${RED}Error: Could not find ROS2 installation${NC}"
        exit 1
    fi
fi

echo "ROS_DISTRO: $ROS_DISTRO"
echo ""

# Check dependencies
echo "Checking dependencies..."
echo ""

check_dependency() {
    if python3 -c "import $1" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1"
        return 1
    fi
}

MISSING_DEPS=0

if ! check_dependency "PyQt5"; then
    echo "  Install with: sudo apt-get install python3-pyqt5"
    MISSING_DEPS=1
fi

if ! check_dependency "psutil"; then
    echo "  Install with: sudo apt-get install python3-psutil"
    MISSING_DEPS=1
fi

if ! check_dependency "matplotlib"; then
    echo "  Install with: sudo apt-get install python3-matplotlib"
    MISSING_DEPS=1
fi

echo ""

if [ $MISSING_DEPS -eq 1 ]; then
    echo -e "${YELLOW}Missing dependencies detected!${NC}"
    echo ""
    read -p "Install missing dependencies now? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Installing dependencies..."
        sudo apt-get update
        sudo apt-get install -y python3-pyqt5 python3-psutil python3-matplotlib
        echo -e "${GREEN}✓ Dependencies installed${NC}"
    else
        echo -e "${RED}Cannot build without dependencies. Exiting.${NC}"
        exit 1
    fi
fi

# Navigate to workspace
cd "$WORKSPACE_DIR"

# Clean previous build (optional)
read -p "Clean previous build? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Cleaning previous build..."
    rm -rf build/ros2_studio install/ros2_studio log/ros2_studio
    echo -e "${GREEN}✓ Cleaned${NC}"
fi
echo ""

# Build the package
echo "Building ros2_studio..."
echo "=========================================="
colcon build --packages-select ros2_studio --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo -e "${GREEN}✓ Build successful!${NC}"
    echo "=========================================="
    echo ""
    echo "To use ROS2 Studio, run:"
    echo ""
    echo -e "  ${YELLOW}source install/setup.bash${NC}"
    echo -e "  ${YELLOW}ros2 studio${NC}"
    echo ""
    echo "Or add to your ~/.bashrc:"
    echo ""
    echo -e "  ${YELLOW}echo 'source $WORKSPACE_DIR/install/setup.bash' >> ~/.bashrc${NC}"
    echo ""
else
    echo ""
    echo "=========================================="
    echo -e "${RED}✗ Build failed!${NC}"
    echo "=========================================="
    echo ""
    echo "Check the error messages above."
    exit 1
fi
