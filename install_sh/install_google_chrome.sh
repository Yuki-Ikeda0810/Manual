#!/bin/bash
# 参考：https://linuxize.com/post/how-to-install-google-chrome-web-browser-on-ubuntu-20-04/
# NEED to be tested (problem with --no-sandbox)
# Check https://github.com/GoogleChrome/lighthouse/issues/378

echo "╔══╣ Install: Google Chrome (STARTING) ╠══╗"
sudo apt-get update

sudo apt-get install -y \
    wget

wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb

sudo apt-get install -y \
    ./google-chrome-stable_current_amd64.deb

rm google-chrome-stable_current_amd64.deb
echo "╚══╣ Install: Google Chrome (FINISHED) ╠══╝"

# Other method
# echo "╔══╣ Install: Google Chrome (STARTING) ╠══╗"
# sudo apt-get update

# sudo apt-get install -y \
#     wget \
#     gnupg \
#     gnupg2 \
#     gnupg1

# sudo sh -c 'echo "deb http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'

# sudo wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -

# sudo apt-get update

# sudo apt-get install -y \
#     google-chrome-stable

# echo "╚══╣ Install: Google Chrome (FINISHED) ╠══╝"