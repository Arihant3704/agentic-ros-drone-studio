#!/usr/bin/env bash

# Exit immediately on errors
set -e

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

echo -e "${CYAN}===================================================================${RESET}"
echo -e "${CYAN}         Agentic ROS 2 + Gazebo + PX4 SITL Setup Tool              ${RESET}"
echo -e "${CYAN}===================================================================${RESET}"

# Get directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# 1. Check for ROS 2 installation
if [ -f /opt/ros/humble/setup.bash ]; then
    echo -e "${GREEN}[*] ROS 2 Humble installation detected at /opt/ros/humble.${RESET}"
    source /opt/ros/humble/setup.bash
    
    # Check if colcon is installed
    if command -v colcon > /dev/null; then
        echo -e "${GREEN}[*] Building local workspace /ros2_ws using colcon...${RESET}"
        cd "$DIR/ros2_ws"
        colcon build --symlink-install
        echo -e "${GREEN}[*] Workspace compiled successfully!${RESET}"
        echo -e "${GREEN}[*] To run the nodes, execute:${RESET}"
        echo -e "${CYAN}    source $DIR/ros2_ws/install/setup.bash${RESET}"
        echo -e "${CYAN}    ros2 launch agentic_drone_control agentic_drone.launch.py${RESET}"
    else
        echo -e "${YELLOW}[!] colcon build tool is not installed. Install it with: sudo apt install python3-colcon-common-extensions${RESET}"
    fi
else
    echo -e "${YELLOW}[!] ROS 2 Humble was not detected at /opt/ros/humble/setup.bash.${RESET}"
    echo -e "${YELLOW}[!] Below are step-by-step instructions to configure a complete simulation stack on your machine:${RESET}"
    
    echo -e "\n${CYAN}--- STEP 1: Install ROS 2 Humble (Ubuntu 22.04 LTS) ---${RESET}"
    echo -e "Run the following commands in your host terminal:"
    echo -e "  sudo apt update && sudo apt install locales"
    echo -e "  sudo locale-gen en_US en_US.UTF-8"
    echo -e "  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
    echo -e "  export LANG=en_US.UTF-8"
    echo -e "  sudo apt install software-properties-common"
    echo -e "  sudo add-apt-repository universe"
    echo -e "  sudo apt update && sudo apt install curl -y"
    echo -e "  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    echo -e "  echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
    echo -e "  sudo apt update"
    echo -e "  sudo apt install ros-humble-desktop-full python3-colcon-common-extensions python3-rosdep -y"

    echo -e "\n${CYAN}--- STEP 2: Install PX4 Autopilot SITL & Gazebo ---${RESET}"
    echo -e "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
    echo -e "  cd PX4-Autopilot"
    echo -e "  # Run PX4 developer setup script (installs Gazebo Gz Sim automatically)"
    echo -e "  bash Tools/setup/ubuntu.sh"
    echo -e "  # Reboot your system after completion"

    echo -e "\n${CYAN}--- STEP 3: Setup Micro XRCE-DDS Agent (Middleware Bridge) ---${RESET}"
    echo -e "PX4 uses uXRCE-DDS to publish state topics to ROS 2. Compile the agent bridge:"
    echo -e "  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git"
    echo -e "  cd Micro-XRCE-DDS-Agent"
    echo -e "  mkdir build && cd build"
    echo -e "  cmake .."
    echo -e "  make"
    echo -e "  sudo make install"
    echo -e "  sudo ldconfig /usr/local/lib/"

    echo -e "\n${CYAN}--- STEP 4: Build this local Workspace ---${RESET}"
    echo -e "  cd $DIR/ros2_ws"
    echo -e "  # Source ROS 2"
    echo -e "  source /opt/ros/humble/setup.bash"
    echo -e "  # Compile the workspace"
    echo -e "  colcon build"

    echo -e "\n${CYAN}--- STEP 5: Launch the Complete Gazebo Simulation ---${RESET}"
    echo -e "  1. Open Terminal 1: Run PX4 + Gazebo Simulation"
    echo -e "     cd PX4-Autopilot"
    echo -e "     make px4_sitl gazebo-classic_default"
    echo -e "  2. Open Terminal 2: Run Micro XRCE-DDS Agent"
    echo -e "     MicroXRCEAgent udp4 -p 8888"
    echo -e "  3. Open Terminal 3: Launch your Agentic Control nodes"
    echo -e "     source $DIR/ros2_ws/install/setup.bash"
    echo -e "     ros2 launch agentic_drone_control agentic_drone.launch.py"
    echo -e "  4. Open Terminal 4: Publish natural language mission commands"
    echo -e "     ros2 topic pub /drone/user_command std_msgs/msg/String \"data: 'Arm the drone, takeoff to 5m, and search for the thermal pad'\" -1"
fi
