#!/bin/bash
# Reference 1: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Reference 2: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
# Reference 3: https://unix.stackexchange.com/questions/177138/how-do-i-test-if-an-item-is-in-a-bash-array

ROS2DISall=("humble" "galactic" "foxy")
ROS2DIS=$1
ROS2DIS=${ROS2DIS//-}


# Check if the ROS2 distribution exists (or if supported)
if [[ "${ROS2DISall[*]}" =~ (^|[[:space:]])"${ROS2DIS}"($|[[:space:]]) ]]
then
    echo "╔══╣ Install: ROS2 ${ROS2DIS} (STARTING) ╠══╗"
else
    echo "Please select one of the following ROS2 Distributions"
    echo ${ROS2DISall[*]}
    echo "For example: bash install_ros2.sh --humble"
    exit
fi


# Add the ROS 2 apt repository
sudo apt update
sudo apt install -y \
    curl \
    gnupg \
    lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


# Add the repository to the sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo ${UBUNTU_CODENAME}) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Installation
sudo apt update

sudo apt install -y \
    ros-${ROS2DIS}-desktop


echo "╚══╣ Install: ROS2 ${ROS2DIS} (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"
