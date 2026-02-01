#!/bin/bash
set -e

# Setup environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Determine script location to find setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RTW_ROOT="$(dirname "$SCRIPT_DIR")"
source $RTW_ROOT/setup.bash
export PATH=$RTW_ROOT/scripts:$PATH

# Configure git
git config --global user.email "test@example.com"
git config --global user.name "Test User"

# Create a workspace
mkdir -p ~/ws/src
export ROS_WS=~/ws
cd ~/ws/src

echo "----------------------------------------------------------------"
echo "Creating description package..."
# Input:
# \n : Is this your workspace? (Yes)
# 1  : Standard package
# 1  : User input maintainer
# Test User : Name
# test@example.com : Email
# 1  : User input license
# Apache-2.0 : License
# 1  : ament_cmake
# \n : Confirmation
# no : Setup repository? (No) - Must use 'no' not 'n'
echo -e "\n1\n1\nTest User\ntest@example.com\n1\nApache-2.0\n1\n\nno\n" | create-new-package.bash my_robot_description "Description package"

echo "----------------------------------------------------------------"
echo "Setting up robot description..."
cd my_robot_description
# Input:
# 1  : xml launch files
# \n : Confirmation
echo -e "1\n\n" | setup-robot-description.bash my_robot 1

echo "----------------------------------------------------------------"
echo "Creating control package..."
cd ~/ws/src
# Same input as above
echo -e "\n1\n1\nTest User\ntest@example.com\n1\nApache-2.0\n1\n\nno\n" | create-new-package.bash my_robot_control "Control package"

echo "----------------------------------------------------------------"
echo "Setting up robot bringup..."
cd my_robot_control
# Input:
# 1  : xml launch files
# \n : Confirmation
echo -e "1\n\n" | setup-robot-bringup.bash my_robot my_robot_description

echo "----------------------------------------------------------------"
echo "Building workspace..."
cd ~/ws
apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

echo "----------------------------------------------------------------"
echo "Testing launch files..."
source install/setup.bash

# Test description launch
echo "Launching my_robot_description load_description.launch.xml..."
timeout 10s ros2 launch my_robot_description load_description.launch.xml || if [ $? -eq 124 ]; then echo "Launch started successfully (timed out as expected)"; else echo "Launch failed"; exit 1; fi

# Test bringup launch (start_offline)
echo "Launching my_robot_control start_offline.launch.xml..."
timeout 10s ros2 launch my_robot_control start_offline.launch.xml || if [ $? -eq 124 ]; then echo "Launch started successfully (timed out as expected)"; else echo "Launch failed"; exit 1; fi

echo "All tests passed!"
