#!/bin/bash
set -e
shopt -s expand_aliases

# Setup environment
source /opt/ros/"$ROS_DISTRO"/setup.bash
# Determine script location to find setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RTW_ROOT="$(dirname "$SCRIPT_DIR")"

# Configure git
git config --global user.email "test@example.com"
git config --global user.name "Test User"

# Install rtwcli
echo "Installing rtwcli..."

# Setup auto-sourcing to enable aliases mechanism in setup.bash if needed
source "$RTW_ROOT"/setup.bash
echo -e "\nyes\nyes\n" | setup-auto-sourcing
source ~/.bashrc

# Remove EXTERNALLY-MANAGED to allow pip install
rm -f /usr/lib/python*/EXTERNALLY-MANAGED

cd "$RTW_ROOT"/rtwcli  # enter the RTW-CLI folder
pip3 install -r requirements.txt --break-system-packages  # since Ubuntu 24.04 is this flag required as we are not using virtual environment
# pip3 install ./rtwcli ./rtw_cmds ./rtw_rocker_extensions
cd -  # go back to the folder where you cloned the RTW
export PATH=$PATH:$HOME/.local/bin

# Create a workspace with RTW CLI
echo "Creating workspace with rtwcli..."
rtw workspace create --ws-name test_ws --ws-folder ~/ws --ros-distro "$ROS_DISTRO"

# Use the workspace to setup environment
# rtw workspace use writes to a tmp file targeting parent PID ($$)
# We must source this file manually in a non-interactive script
rtw ws test_ws
source "/tmp/ros_team_workspace/workspace_$$.bash"

echo "----------------------------------------------------------------"
echo "Creating description package..."
rosds
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
echo -e "\n1\n1\nTest User\ntest@example.com\n1\nApache-2.0\n1\n\nno\n" | create-new-package my_robot_description "Description package"

echo "----------------------------------------------------------------"
echo "Setting up robot description..."
cd my_robot_description
# Input:
# 1  : xml launch files
# \n : Confirmation
echo -e "1\n\n" | setup-robot-description my_robot 1

echo "----------------------------------------------------------------"
echo "Creating control package..."

# Same input as above
rosds
echo -e "\n1\n1\nTest User\ntest@example.com\n1\nApache-2.0\n1\n\nno\n" | create-new-package my_robot_control "Control package"

echo "----------------------------------------------------------------"
echo "Setting up robot bringup..."
cd my_robot_control
# Input:
# 1  : xml launch files
# \n : Confirmation
echo -e "1\n\n" | setup-robot-bringup my_robot my_robot_description

echo "----------------------------------------------------------------"
echo "Building workspace..."
rosdep_prep
rosdepi
cb

echo "----------------------------------------------------------------"
echo "Testing launch files..."
# Refresh environment after build
rtw ws test_ws
source "/tmp/ros_team_workspace/workspace_$$.bash"

# Function to kill background process on exit or error
cleanup() {
  if [ -n "$PID_DESC" ]; then kill "$PID_DESC" 2>/dev/null || true; fi
  if [ -n "$PID_CTRL" ]; then kill "$PID_CTRL" 2>/dev/null || true; fi
}
trap cleanup EXIT

# Test description launch
echo "Launching my_robot_description load_description.launch.xml..."
ros2 launch my_robot_description load_description.launch.xml &
PID_DESC=$!
echo "Waiting for launch file to start..."
sleep 10

echo "Checking for /robot_description topic..."
if ros2 topic list | grep -q "/robot_description"; then
    echo -e "${TERMINAL_COLOR_GREEN}Topic /robot_description found.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Topic /robot_description not found.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "Checking for data on /robot_description..."
if ros2 topic echo /robot_description --once --timeout 5 > /dev/null; then
    echo -e "${TERMINAL_COLOR_GREEN}Data received on /robot_description.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: No data received on /robot_description.${TERMINAL_COLOR_NC}"
    exit 1
fi

kill $PID_DESC
PID_DESC=""
sleep 5

# Test bringup launch (start_offline)
echo "Launching my_robot_control start_offline.launch.xml..."
ros2 launch my_robot_control start_offline.launch.xml &
PID_CTRL=$!
echo "Waiting for launch file to start..."
sleep 20

echo "Checking controllers..."
CONTROLLER_LIST=$(ros2 control list_controllers)
echo "$CONTROLLER_LIST"

ACTIVE_COUNT=$(echo "$CONTROLLER_LIST" | grep "active" | wc -l)
echo "Found $ACTIVE_COUNT active controllers."

if [ "$ACTIVE_COUNT" -ge 2 ]; then
    echo "Success: At least 2 controllers are active."
else
    echo "Error: Expected at least 2 active controllers, found $ACTIVE_COUNT."
    exit 1
fi

kill $PID_CTRL
PID_CTRL=""

echo "All tests passed!"
