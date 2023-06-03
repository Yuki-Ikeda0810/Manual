#!/bin/bash
# Reference 1: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

ROS2DISall=("humble" "galactic" "foxy")
ROS2DIS=$1
ROS2DIS=${ROS2DIS//-}

# Check if the ROS1 distribution exists (or if supported)
if [[ "${ROS2DISall[*]}" =~ (^|[[:space:]])"${ROS2DIS}"($|[[:space:]]) ]]
then
    echo "╔══╣ Install: ROS2 ${ROS2DIS} Workspace (STARTING) ╠══╗"
else
    echo "Please select one of the following ROS2 Distributions"
    echo ${ROS2DISall[*]}
    echo "For example: bash install_colcon_ws.sh --humble"
    exit
fi

source /opt/ros/${ROS2DIS}/setup.bash
source ~/.bashrc

sudo apt-get update
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep2

rosdep update

# Create a new directory for the workspace
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

# Clone a sample repository
git clone https://github.com/ros/ros_tutorials.git -b ${ROS2DIS}-devel

# Resolve dependencies
cd ~/colcon_ws
rosdep install -i --from-path src --rosdistro ${ROS2DIS} -y

# Build the workspace with colcon
colcon build


echo "╚══╣ Install: ROS2 ${ROS2DIS} Workspace (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"