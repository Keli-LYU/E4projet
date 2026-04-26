#!/bin/bash
# Script for running simulation tests using Carla simulator

set -e # Exit on error

echo "Starting simulation tests..."

# Source ROS environment
source /opt/ros/galactic/setup.bash
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
fi

# Start Carla simulator in background
echo "Starting Carla simulator..."
/opt/Carla/CARLA_0.9.13/CarlaUE4.sh -opengl &
CARLA_PID=$!

# Give Carla time to start
sleep 10

# Start ROS bridge in background
echo "Starting ROS bridge..."
ros2 launch expleo_nav_stack carla_bridge_bringup.launch.py &
BRIDGE_PID=$!

# Give bridge time to connect
sleep 5

# Run a simple navigation test
echo "Running navigation test..."
# Launch navigation stack with test parameters
ros2 launch expleo_nav_stack expleo_navstack_bringup.launch.py \
  map:=./src/expleo_nav_stack/maps/Town10_xodr.yaml \
  params_file:=./src/expleo_nav_stack/expleo_nav_stack/params/nav2_config_smac_hybrid__regulated_pure_pursuit__carla_yolov7.yaml \
  use_sim_time:=true \
  autostart:=true &
NAV_PID=$!

# Give navigation stack time to start
sleep 10

# Send a navigation goal
echo "Sending navigation goal..."
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {
    frame_id: 'map'
  },
  pose: {
    position: {
      x: 10.0,
      y: 10.0,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    }
  }
}"

# Wait for navigation to complete or timeout
TIMEOUT=60
echo "Waiting for navigation to complete (timeout: ${TIMEOUT}s)..."
for ((i=1; i<=$TIMEOUT; i++)); do
  # Check if we've reached the goal
  # This is a simple example - in reality, you would check the actual robot position
  GOAL_REACHED=$(ros2 topic echo --once /goal_reached std_msgs/Bool 2>/dev/null || echo "false")
  if [[ "$GOAL_REACHED" == *"data: true"* ]]; then
    echo "Goal reached successfully!"
    break
  fi
  
  # If we've reached the timeout, consider it a failure
  if [ $i -eq $TIMEOUT ]; then
    echo "Navigation test failed: timeout reached"
    FAILED=1
  fi
  
  sleep 1
  echo -n "."
done

# Clean up processes
echo "Cleaning up..."
kill $NAV_PID 2>/dev/null || true
kill $BRIDGE_PID 2>/dev/null || true
kill $CARLA_PID 2>/dev/null || true

# Wait for processes to terminate
sleep 5

# Report test results
if [ -z "$FAILED" ]; then
  echo "Simulation test completed successfully!"
  exit 0
else
  echo "Simulation test failed!"
  exit 1
fi
