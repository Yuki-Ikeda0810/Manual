#!/bin/bash
# Reference 1: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

ROS2DISall=("humble" "galactic" "foxy")
ROS2DIS=$1
ROS2DIS=${ROS2DIS//-}


# Check if the ROS2 distribution exists (or if supported)
if [[ "${ROS2DISall[*]}" =~ (^|[[:space:]])"${ROS2DIS}"($|[[:space:]]) ]]
then
    echo "╔══╣ Set-Up: ROS2 ${ROS2DIS} environment (STARTING) ╠══╗"
else
    echo "Please select one of the following ROS2 Distributions"
    echo ${ROS2DISall[*]}
    echo "For example: bash install_ros2.sh --humble"
    exit
fi


echo "source /opt/ros/${ROS2DIS}/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
# echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/${ROS2DIS}/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc

source ~/.bashrc

echo `printenv | grep -i ROS`

echo "╚══╣ Set-Up: ROS2 environment (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"