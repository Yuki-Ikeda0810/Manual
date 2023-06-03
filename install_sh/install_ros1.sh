#!/bin/bash
# 参考①：http://wiki.ros.org/noetic/Installation/Ubuntu
# 参考②：https://unix.stackexchange.com/questions/177138/how-do-i-test-if-an-item-is-in-a-bash-array

ROS1DISall=("noetic" "melodic" "indigo" "kinetic")
ROS1DIS=$1
ROS1DIS=${ROS1DIS//-}

# Check if the ROS1 distribution exists (or if supported)
if [[ "${ROS1DISall[*]}" =~ (^|[[:space:]])"${ROS1DIS}"($|[[:space:]]) ]]
then
    echo "╔══╣ Install: ROS $ROS1DIS (STARTING) ╠══╗"
else
    echo "Please select one of the following ROS1 Distrubutions"
    echo ${ROS1DISall[*]}
    echo "For example: bash install_ros1.sh --noetic"
    exit
fi

# Setup your sources.list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install -y \
    curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation
sudo apt update

sudo apt install -y \
    ros-$ROS1DIS-desktop-full

# Environment setup
echo "source /opt/ros/$ROS1DIS/setup.bash" >> ~/.bashrc
source ~/.bashrc


# Dependencies for building packages
if [[ $ROS1DIS != "noetic" ]]
then
    sudo apt install -y \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential

    sudo apt install -y \
        python-rosdep
else
    sudo apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential

    sudo apt install -y \
        python3-rosdep
fi

sudo rosdep init
rosdep update

echo "╚══╣ Install: ROS $ROS1DIS (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"
