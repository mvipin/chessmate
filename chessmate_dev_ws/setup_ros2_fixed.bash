#!/bin/bash
# Fixed ROS2 environment setup

# Source virtual environment for Python dependencies
VENV_PATH="${HOME}/ChessMate-ROS2/chessmate_env"
if [ -f "${VENV_PATH}/bin/activate" ]; then
    source "${VENV_PATH}/bin/activate"
    # Also export PYTHONPATH for subprocesses that may not inherit venv
    export PYTHONPATH="${VENV_PATH}/lib/python3.12/site-packages:${PYTHONPATH}"
fi

# Auto-detect ROS distro (prefer jazzy, fallback to humble)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    export ROS_DISTRO=jazzy
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    export ROS_DISTRO=humble
else
    echo "❌ No ROS2 installation found!"
    return 1
fi

# Set missing environment variables if needed
export ROS_VERSION=2

# Set default RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source workspace - check multiple possible locations
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "install_arm/setup.bash" ]; then
    source install_arm/setup.bash
fi

echo "✅ ROS2 environment fixed and ready"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "PYTHONPATH includes venv: $(echo $PYTHONPATH | grep -c chessmate_env > /dev/null && echo 'yes' || echo 'no')"
