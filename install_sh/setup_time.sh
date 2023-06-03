#!/bin/bash 
# 参考①：https://netlog.jpn.org/r271-635/2021/01/ubuntu_rtc_local_tz.html
# 参考②：https://www.yokoweb.net/2018/05/14/ubuntu-18_04-timesyncd/
# 参考③：https://itsfoss.com/wrong-time-dual-boot/

echo "╔══╣ Set-Up: Time Synchronization (STARTING) ╠══╗"


# Fix the time discrepancy between Ubuntu and Windows
sudo timedatectl set-local-rtc 1

# Fix time server
cat /etc/systemd/timesyncd.conf | sudo sed -i -e 's/#NTP=/NTP=time.soka.ac.jp/g' > sudo /etc/systemd/timesyncd.conf
cat /etc/systemd/timesyncd.conf | sudo sed -i -e 's/#FallbackNTP=ntp.ubuntu.com/FallbackNTP=ntp.nict.jp/g' > sudo /etc/systemd/timesyncd.conf

# Restart: systemd-timesyncd.service
sudo systemctl restart systemd-timesyncd.service

# Set Hardware Clock by default
timedatectl set-local-rtc 1


echo "╚══╣ Set-Up: Time Synchronization (FINISHED) ╠══╝"

# --- Other method ---
# # install ntp
# sudo apt install -y ntp
# ​
# # fix ntp.conf
# sudo sed -i -e "21,24s/^/#/" /etc/ntp.conf
# sudo sed -i -e "27s/^/#/" /etc/ntp.conf
# sudo sed -i -e "29iserver ntp.nict.jp" /etc/ntp.conf
# sudo sed -i -e "29iserver time.soka.ac.jp" /etc/ntp.conf
# ​
# # restart ntp
# sudo service ntp restart
# ​
# ntpq -p 
# ​
# echo "上にNICTと創価大のNTPサーバーが追加されていたらOK！"
# ​
# echo "再起動して下さい"