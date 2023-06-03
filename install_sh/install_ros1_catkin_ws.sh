#!/bin/bash
# 参考：wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

ROS1DISall=("noetic" "melodic" "indigo" "kinetic")
ROS1DIS=$1
ROS1DIS=${ROS1DIS//-}

# Check if the ROS1 distribution exists (or if supported)
if [[ "${ROS1DISall[*]}" =~ (^|[[:space:]])"${ROS1DIS}"($|[[:space:]]) ]]
then
    echo "╔══╣ Install: ROS $ROS1DIS Workspace (STARTING) ╠══╗"
else
    echo "Please select one of the following ROS1 Distrubutions"
    echo ${ROS1DISall[*]}
    echo "For example: bash install_catkin_ws.sh --noetic"
    exit
fi

source /opt/ros/$ROS1DIS/setup.bash
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "╚══╣ Install: ROS $ROS1DIS Workspace (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"