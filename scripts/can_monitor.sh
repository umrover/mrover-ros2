#!/bin/bash

if [ -z "$1" ]; then
    echo "Error: Missing CAN interface argument."
    echo "Usage: $0 <interface> [poll_interval]"
    echo "Example: $0 can0"
    echo "Example: $0 can1 5"
    exit 1
fi

NETWORK="$1"
POLL_INTERVAL="${2:-1}" 
ROS_PID=""

start_ros_node() {
    echo "Launching ROS 2 jetson_can node..."
    # Run in the background using '&'
    ros2 launch mrover launch jetson_can.launch.py &
    
    # Capture the Process ID of the last background command
    ROS_PID=$!
    echo "ROS 2 node running (PID: $ROS_PID)."
}

stop_ros_node() {
    if [ -n "$ROS_PID" ]; then
        echo "Killing ROS 2 node (PID: $ROS_PID)..."
        kill -INT "$ROS_PID" 2>/dev/null
        
        wait "$ROS_PID" 2>/dev/null 
        ROS_PID=""
    fi
}

trap "echo -e '\nStopping monitor...'; stop_ros_node; exit 0" SIGINT SIGTERM

start_ros_node

echo "Monitoring $NETWORK every $POLL_INTERVAL seconds..."

while true; do
    # Check if the can bus is DOWN
    if ip -d link show "$NETWORK" | grep -q -e "state DOWN"; then
        echo "---------------------------------------------------"
        echo "$(date +'%Y-%m-%d %H:%M:%S') - $NETWORK went DOWN!"
        
        echo "Executing restart_can..."
        sudo restart_can
        
        stop_ros_node
        
        echo "Waiting 5 seconds for hardware to stabilize..."
        sleep 5
        
        start_ros_node
        echo "Recovery complete. Resuming monitoring..."
        echo "---------------------------------------------------"
    fi
    
    sleep "$POLL_INTERVAL"
done