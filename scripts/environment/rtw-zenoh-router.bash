#!/bin/bash
# Script to start a local zenoh router, optionally connecting to a specific IP.

# load defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source "$script_own_dir/../../setup.bash"

show_help() {
    echo -e "${TERMINAL_COLOR_USER_NOTICE}Usage: rtw-zenoh-router [IP_ADDRESS]${TERMINAL_COLOR_NC}"
    echo -e "Starts a local Zenoh router (rmw_zenohd).\n"
    
    echo -e "${TERMINAL_COLOR_LIGHT_BLUE}Modes:${TERMINAL_COLOR_NC}"
    echo -e "  ${TERMINAL_COLOR_GREEN}Local-only:${TERMINAL_COLOR_NC}  rtw-zenoh-router        (Local nodes only)"
    echo -e "  ${TERMINAL_COLOR_GREEN}Connected:${TERMINAL_COLOR_NC}   rtw-zenoh-router <IP>   (Connects to other router on tcp/<IP>:7447)\n"
    
    echo -e "${TERMINAL_COLOR_LIGHT_BLUE}Options:${TERMINAL_COLOR_NC}"
    echo "  -h, --help   Show this help message."
}
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    show_help
    exit 0
fi

# Check if RMW_IMPLEMENTATION is correctly set
if [[ "$RMW_IMPLEMENTATION" != "rmw_zenoh_cpp" ]]; then
    print_and_exit $'RMW_IMPLEMENTATION is not set to \'rmw_zenoh_cpp\'. Please export it in your workspace configuration (.ros_team_ws_rc) first.\n\nTutorial on: https://rtw.b-robotized.com/master/rtwcli/index.html#modifying-an-existing-workspace\n\n'
fi

# Ensure log variables are exported (fallback if not set in the workspace)
export RUST_LOG=${RUST_LOG:-"zenoh=warn,zenoh_transport=warn"}

# optional IP address argument
if [ -n "$1" ]; then
    TARGET_IP="$1"
    notify_user "Starting Zenoh router and connecting to TCP endpoint: ${TARGET_IP}:7447"
    export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/${TARGET_IP}:7447\"]"
else
    notify_user "Starting Zenoh router..."
    unset ZENOH_CONFIG_OVERRIDE
fi

ros2 run rmw_zenoh_cpp rmw_zenohd