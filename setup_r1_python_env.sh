#!/bin/bash
# Setup script for R1 Pro Python development environment

# === CONFIGURATION ===
# Change this to your actual SDK installation path
R1_SDK_PATH="${HOME}/galaxea_r1_sdk"

# === SOURCE ROS 2 ===
# Source base ROS 2 installation first
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ Sourced ROS 2 Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ Sourced ROS 2 Humble"
else
    echo "❌ ROS 2 not found. Please install ROS 2 first."
    exit 1
fi

# === SOURCE R1 PRO WORKSPACE ===
if [ -f "${R1_SDK_PATH}/install/setup.bash" ]; then
    source "${R1_SDK_PATH}/install/setup.bash"
    echo "✅ Sourced R1 Pro SDK from ${R1_SDK_PATH}"
else
    echo "❌ R1 Pro SDK not found at ${R1_SDK_PATH}"
    echo "   Please set R1_SDK_PATH to your installation directory"
    exit 1
fi

# === VERIFY HDAS MESSAGE PACKAGE ===
echo ""
echo "Checking hdas_msg package..."
ros2 pkg list | grep -q hdas_msg && echo "✅ hdas_msg package found" || echo "❌ hdas_msg package NOT found"

# === SHOW MESSAGE DEFINITION ===
echo ""
echo "MotorControl message definition:"
echo "================================"
ros2 interface show hdas_msg/msg/MotorControl 2>/dev/null || echo "Unable to show message definition"

# === LIST AVAILABLE TOPICS ===
echo ""
echo "Available motion_control topics:"
echo "================================="
ros2 topic list 2>/dev/null | grep motion_control || echo "(Driver not running - start with: ros2 launch HDAS r1pro.py)"

echo ""
echo "✅ Environment ready! You can now run Python scripts."
echo "   Example: python3 r1_motor_control_example.py"
