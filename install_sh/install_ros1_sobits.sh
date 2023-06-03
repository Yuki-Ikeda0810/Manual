#!/bin/bash
# 参考：wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

ROS1DISall=("noetic" "melodic" "indigo" "kinetic")
ROS1DIS=$1
ROS1DIS=${ROS1DIS//-}

# Check if the ROS1 distribution exists (or if supported)
if [[ "${ROS1DISall[*]}" =~ (^|[[:space:]])"${ROS1DIS}"($|[[:space:]]) ]]
then
    echo "╔══╣ Install: ROS $ROS1DIS Packages for SOBITS (STARTING) ╠══╗"
else
    echo "Please select one of the following ROS1 Distrubutions"
    echo ${ROS1DISall[*]}
    echo "For example: bash install_catkin_ws.sh --noetic"
    exit
fi

# Install ROS Common Packages
sudo apt-get update

# Depricated: ros-melodic-uvc-camera
sudo apt-get install -yq \
    ros-$ROS1DIS-rosbridge-* \
    ros-$ROS1DIS-ecl-* \
    ros-$ROS1DIS-joy \
    ros-$ROS1DIS-roswww \
    ros-$ROS1DIS-kobuki-* \
    ros-$ROS1DIS-joint-state-publisher* \
    ros-$ROS1DIS-usb-cam \
    ros-$ROS1DIS-libuvc-camera \
    ros-$ROS1DIS-camera-calibration \
    ros-$ROS1DIS-image-proc \
    ros-$ROS1DIS-image-view \
    libsensors4-dev


# Install PCL
sudo apt-get install -yq \
    ros-$ROS1DIS-pcl-* \
    ros-$ROS1DIS-openni2-*


# Install joint
sudo apt-get install -yq \
    ros-$ROS1DIS-joint*


# Install ps4 controller
sudo apt-get install -yq \
    ros-$ROS1DIS-joy


# Install Pybind
sudo apt-get install -yq \
    ros-$ROS1DIS-pybind11-catkin


# Install URG
sudo apt-get install -yq \
    ros-$ROS1DIS-urg-node


# Install SLAM
sudo apt-get install -yq \
    ros-$ROS1DIS-map-server \
    ros-$ROS1DIS-move-base-* \
    ros-$ROS1DIS-dwa-local-planner


# Install text to speech library
sudo apt-get install -yq \
    mpg321 \
    open-jtalk \
    open-jtalk-mecab-naist-jdic \
    hts-voice-nitech-jp-atr503-m001 \
    libttspico-utils \
    alsa \
    alsa-utils \
    python3-soundfile \
    pulseaudio


# Install sound library
python3 -m pip install \
    gTTS==2.0.1 \
    wave \
    mutagen \
    soundfile

python3 -m pip install -U \
    gTTS-token


echo "╚══╣ Install: ROS $ROS1DIS Packages for SOBITS (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"
source ~/.bashrc