#!/bin/bash

echo "╔══╣ Set-Up: ROS1 environment (STARTING) ╠══╗"


echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/\${USER}/catkin_ws" >> /home/${USER}/.bashrc
echo "export ROS_WORKSPACE=/home/\${USER}/catkin_ws" >> /home/${USER}/.bashrc
echo "export IFACE=\`ifconfig | head -n 1 | awk '{ print \$1}' | rev | cut -b 2- | rev\`" >> /home/${USER}/.bashrc
echo -e "export LOCAL_IP=172.16.10.2\nif [[ \`hostname -I\` =~ .*\$LOCAL_IP.* ]]; then\n  export ROS_IP=\$LOCAL_IP\nelse\n  export ROS_IP=\`hostname -I | cut -d' ' -f1\`\nfi" >> /home/${USER}/.bashrc
echo "export ROS_MASTER_URI=http://\$ROS_IP:11311" >> /home/${USER}/.bashrc
echo "export NO_AT_BRIDGE=1" >> /home/${USER}/.bashrc


echo "╚══╣ Set-Up: ROS1 environment (FINISHED) ╠══╝"
echo "source ~/.bashrc をして下さい"
source ~/.bashrc