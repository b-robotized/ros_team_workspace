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
echo -e "${TERMINAL_COLOR_BLUE}Installing rtwcli...${TERMINAL_COLOR_NC}"

# Setup auto-sourcing to enable aliases mechanism in setup.bash if needed
source "$RTW_ROOT"/setup.bash
echo -e "\nyes\nyes\n" | setup-auto-sourcing
source ~/.bashrc

# Remove EXTERNALLY-MANAGED to allow pip install
rm -f /usr/lib/python*/EXTERNALLY-MANAGED

cd "$RTW_ROOT"/rtwcli  # enter the RTW-CLI folder
pip3 install -r requirements.txt --break-system-packages
cd -  # go back to the folder where you cloned the RTW
export PATH=$PATH:$HOME/.local/bin

# Helper: switch to a workspace and source the generated env file
use_workspace() {
  local ws_name=$1
  rtw ws "$ws_name"
  source "/tmp/ros_team_workspace/workspace_$$.bash"
}

echo "================================================================"
echo -e "${TERMINAL_COLOR_BLUE}Testing RTW CLI workspace and environment variable commands${TERMINAL_COLOR_NC}"
echo "================================================================"

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Creating two workspaces...${TERMINAL_COLOR_NC}"

rtw workspace create --ws-name ws_alpha --ws-folder ~/ws_alpha --ros-distro "$ROS_DISTRO"
rtw workspace create --ws-name ws_beta --ws-folder ~/ws_beta --ros-distro "$ROS_DISTRO"

echo -e "${TERMINAL_COLOR_GREEN}Success: Created workspaces ws_alpha and ws_beta.${TERMINAL_COLOR_NC}"

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Setting different ROS_DOMAIN_ID on each workspace...${TERMINAL_COLOR_NC}"

rtw workspace edit ws_alpha --env-vars ROS_DOMAIN_ID=10
rtw workspace edit ws_beta --env-vars ROS_DOMAIN_ID=20

echo -e "${TERMINAL_COLOR_GREEN}Success: Set ROS_DOMAIN_ID=10 on ws_alpha and ROS_DOMAIN_ID=20 on ws_beta.${TERMINAL_COLOR_NC}"

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Test 1: Switching to ws_alpha and checking ROS_DOMAIN_ID...${TERMINAL_COLOR_NC}"

use_workspace ws_alpha

echo -e "${TERMINAL_COLOR_BLUE}ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${TERMINAL_COLOR_NC}"
if [ "$ROS_DOMAIN_ID" = "10" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: ROS_DOMAIN_ID is 10 in ws_alpha.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Expected ROS_DOMAIN_ID=10 in ws_alpha, got '${ROS_DOMAIN_ID}'.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Test 2: Switching to ws_beta and checking ROS_DOMAIN_ID...${TERMINAL_COLOR_NC}"

use_workspace ws_beta

echo -e "${TERMINAL_COLOR_BLUE}ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${TERMINAL_COLOR_NC}"
if [ "$ROS_DOMAIN_ID" = "20" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: ROS_DOMAIN_ID is 20 in ws_beta.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Expected ROS_DOMAIN_ID=20 in ws_beta, got '${ROS_DOMAIN_ID}'.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Test 3: Switching back to ws_alpha to confirm isolation...${TERMINAL_COLOR_NC}"

use_workspace ws_alpha

echo -e "${TERMINAL_COLOR_BLUE}ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${TERMINAL_COLOR_NC}"
if [ "$ROS_DOMAIN_ID" = "10" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: ROS_DOMAIN_ID correctly restored to 10 when switching back to ws_alpha.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Expected ROS_DOMAIN_ID=10 after switching back to ws_alpha, got '${ROS_DOMAIN_ID}'.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Test 4: Updating ROS_DOMAIN_ID on ws_alpha using rtw workspace edit...${TERMINAL_COLOR_NC}"

rtw workspace edit ws_alpha --env-vars ROS_DOMAIN_ID=99

# Re-source the workspace to pick up the change
use_workspace ws_alpha

echo -e "${TERMINAL_COLOR_BLUE}ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${TERMINAL_COLOR_NC}"
if [ "$ROS_DOMAIN_ID" = "99" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: ROS_DOMAIN_ID updated to 99 on ws_alpha.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Expected ROS_DOMAIN_ID=99 after update, got '${ROS_DOMAIN_ID}'.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Test 5: Removing ROS_DOMAIN_ID from ws_alpha...${TERMINAL_COLOR_NC}"

rtw workspace edit ws_alpha --remove-env-vars ROS_DOMAIN_ID

# Set a known value first to verify it gets cleared/overridden by the workspace source
export ROS_DOMAIN_ID=999
use_workspace ws_alpha

echo -e "${TERMINAL_COLOR_BLUE}ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${TERMINAL_COLOR_NC}"
# After removal, the workspace should no longer export ROS_DOMAIN_ID
# so the sourced workspace file should not set it, and the value from
# the environment before sourcing (999) should remain
# BUT the key test is that it's NOT set to 99 (the old workspace value)
if [ "$ROS_DOMAIN_ID" != "99" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: ROS_DOMAIN_ID is no longer set by ws_alpha (value: '${ROS_DOMAIN_ID}').${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: ROS_DOMAIN_ID still set to removed value 99.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Test 6: ws_beta still has its own ROS_DOMAIN_ID after ws_alpha changes...${TERMINAL_COLOR_NC}"

use_workspace ws_beta

echo -e "${TERMINAL_COLOR_BLUE}ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${TERMINAL_COLOR_NC}"
if [ "$ROS_DOMAIN_ID" = "20" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: ws_beta ROS_DOMAIN_ID still 20, unaffected by ws_alpha changes.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Expected ROS_DOMAIN_ID=20 in ws_beta, got '${ROS_DOMAIN_ID}'.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "================================================================"
echo -e "${TERMINAL_COLOR_GREEN}All RTW CLI tests passed!${TERMINAL_COLOR_NC}"
