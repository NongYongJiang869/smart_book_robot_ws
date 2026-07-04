#!/bin/bash
# 等待 lifecycle 节点就绪后自动激活
set -e

wait_and_activate() {
    local node=$1
    local state="unknown"
    local waited=0
    while [ "$state" != "unconfigured" ] && [ "$state" != "inactive" ]; do
        state=$(ros2 lifecycle get $node 2>/dev/null | head -1 | awk '{print $1}' || echo "unknown")
        if [ "$state" == "active" ]; then
            echo "$node already active"
            return 0
        fi
        sleep 0.5
        waited=$((waited+1))
        if [ $waited -gt 30 ]; then
            echo "Timeout waiting for $node"
            return 1
        fi
    done
    echo "Configuring $node..."
    ros2 lifecycle set $node configure
    sleep 0.5
    echo "Activating $node..."
    ros2 lifecycle set $node activate
    echo "$node activated"
}

echo "Waiting for nodes..."
sleep 3
wait_and_activate /map_server
wait_and_activate /amcl
echo "Done!"
