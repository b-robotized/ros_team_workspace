export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# increase log level to see the connection happening. Try info, debug or trace.
export RUST_LOG=zenoh=warn,zenoh_transport=warn
# Connect to your CtrlX device IP
#export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.28.28:7447"]'
# Start the router and leave it running.
ros2 run rmw_zenoh_cpp rmw_zenohd
