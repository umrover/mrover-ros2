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

echo "Monitoring $NETWORK every $POLL_INTERVAL seconds..."

while true; do
    # Check if the can bus is DOWN
    if ip -d link show "$NETWORK" | grep -q -e "state DOWN" -e "bus-off"; then
        echo "$(date +'%Y-%m-%d %H:%M:%S') - Warning: $NETWORK is DOWN or BUS-OFF."
        echo "Attempting recovery..."
        
        # Run your custom restart command
        sudo restart_can
        
        # Give the interface a few seconds to come back up before polling again
        sleep 5
    fi
    
    sleep "$POLL_INTERVAL"
done