#!/bin/bash 
# 参考①：https://gist.github.com/vratiu/9780109

echo "╔══╣ Set-Up: Alias (STARTING) ╠══╗"

NAME=sobits
ALIAS_PWD=$(find ~/ -maxdepth 1 -name ".bash_aliases")

if [ -z "${ALIAS_PWD}" ]; then
    touch ~/.bash_aliases
    echo "source ~/.bash_aliases" >> /home/${USER}/.bashrc
    echo "" >> /home/${USER}/.bashrc
fi

# build
echo "# build" >> /home/${USER}/.bash_aliases
echo "alias cmd='python3 /home/\${USER}/sobits_manual_mini/install_sh/chmod_all.py'" >> /home/${USER}/.bash_aliases
echo "alias cmk='CULLENT_DIR=`pwd` && cd /home/\${USER}/catkin_ws/ && catkin_make -j\$[\$(grep cpu.cores /proc/cpuinfo | sort -u | sed 's/[^0-9]//g') + 1] -DCMAKE_CXX_FLAGS=-O3 && cd $CULLENT_DIR'" >> /home/${USER}/.bash_aliases
echo "alias cm='cmk && cmd'" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases

# pip
echo "# pip" >> /home/${USER}/.bash_aliases
echo "alias pip='python -m pip'" >> /home/${USER}/.bash_aliases
echo "alias pip2='python2 -m pip' " >> /home/${USER}/.bash_aliases
echo "alias pip3='python3 -m pip'" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases

# apt
echo "# apt" >> /home/${USER}/.bash_aliases
echo "alias agi='sudo apt install'" >> /home/${USER}/.bash_aliases
echo "alias agr='sudo apt remove'" >> /home/${USER}/.bash_aliases
echo "alias agu='sudo apt update'" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases

# ls
echo "# ls" >> /home/${USER}/.bash_aliases
echo "alias ls='ls --color=auto'" >> /home/${USER}/.bash_aliases
echo "alias ll='ls -alF'" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases

# cd
echo "# cd" >> /home/${USER}/.bash_aliases
echo "alias cdc='cd ~/catkin_ws/src'" >> /home/${USER}/.bash_aliases
echo "alias ..='cd ..'" >> /home/${USER}/.bash_aliases
echo "alias ...='cd ../..'" >> /home/${USER}/.bash_aliases
echo "alias ....='cd ../../..'" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases

# cp,mv,rm
echo "# cp,mv,rm" >> /home/${USER}/.bash_aliases
echo "alias cp='cp -i'" >> /home/${USER}/.bash_aliases
echo "alias mv='mv -i'" >> /home/${USER}/.bash_aliases
echo "alias rm='rm -i'" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases

# git
echo "# git" >> /home/${USER}/.bash_aliases
echo "alias g='git'" >> /home/${USER}/.bash_aliases
echo "alias ga='git add'" >> /home/${USER}/.bash_aliases
echo "alias gd='git diff'" >> /home/${USER}/.bash_aliases
echo "alias gs='git status'" >> /home/${USER}/.bash_aliases
echo "alias gp='git push'" >> /home/${USER}/.bash_aliases
echo "alias gb='git branch'" >> /home/${USER}/.bash_aliases
echo "alias gst='git status'" >> /home/${USER}/.bash_aliases
echo "alias gco='git checkout'" >> /home/${USER}/.bash_aliases
echo "alias gf='git fetch'" >> /home/${USER}/.bash_aliases
echo "alias gc='git commit'" >> /home/${USER}/.bash_aliases
echo "function gcs() {
    command git clone https://github.com/TeamSOBITS/$1.git 
}" >> /home/${USER}/.bash_aliases
echo "" >> /home/${USER}/.bash_aliases


if [ $USER != $NAME ]; 
then
    # docker(Host Only)
    echo "# docker" >> /home/${USER}/.bash_aliases
    echo "alias ce='python3 ~/docker_ws/container_executer.py'" >> /home/${USER}/.bash_aliases
    echo "alias noetic_sobits_ws='docker exec -it noetic_sobits_ws bash'" >> /home/${USER}/.bash_aliases
    echo "alias melodic_sobit_ws='docker exec -it melodic_sobit_ws bash'" >> /home/${USER}/.bash_aliases
    echo "" >> /home/${USER}/.bash_aliases
fi


echo "export PYTHONDONTWRITEBYTECODE=1" >> /home/${USER}/.bash_aliases
echo "sudo chmod 777 /dev/video*" >> /home/${USER}/.bash_aliases
echo "export PS1='\[\e[1;33;40m\]\$CONTAINER_NAME\[\e[0m\] \${debian_chroot:+(\$debian_chroot)}\[\033[01;32m\]\u@\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '">> /home/${USER}/.bash_aliases
echo "export EDITOR='nano'" >> /home/${USER}/.bash_aliases

echo "╚══╣ Set-Up: Alias (FINISHED) ╠══╝"
