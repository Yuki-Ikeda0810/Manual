#!/bin/bash
# Reference 1: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
# Reference 2: https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-20-04-focal-fossa-linux
# Reference 3: https://developer.nvidia.com/cuda-toolkit-archive
# Reference 4: https://developer.nvidia.com/rdp/cudnn-archive
# Reference 5: https://askubuntu.com/questions/1162491/how-can-you-tell-the-version-of-ubuntu-on-a-system-in-a-sh-bash-script
# Reference 6: https://docs.nvidia.com/deploy/cuda-compatibility/index.html#deployment-consideration-forward

CUDA_LIST=("11.8.0"    "11.7.1"    "11.6.2"    "11.6.1"    "11.6.0"    "11.3.0")
CUDN_LIST=("8.7.0.84" "8.5.0.96"  "8.4.1.50"  "8.4.0.27"  "8.4.0.27"  "8.2.1.32")
NV_DRIVER=("520.61.05" "515.65.01" "510.47.03" "510.47.03" "510.39.01" "465.19.01")
NV_NO="TBD"
NV_VER="TBD"
CUDA_VER=$1
CUDA_VER=${CUDA_VER//-}
CUDNN_VER="TBD"
UB_VER=$(cut -f2 <<< "`lsb_release -r`")
UB_VER=${UB_VER//.}

# Check if the CUDA version to install exists (or if supported)
if [[ "${CUDA_LIST[*]}" =~ (^|[[:space:]])"${CUDA_VER}"($|[[:space:]]) ]]
then
    # Check NVIDIA driver for the specified CUDA version
    cnt=0
    for i in "${CUDA_LIST[@]}"
    do
        NV_NO=$cnt
        # echo "${i::-2} $CUDA_VER"
        if [[ " ${i::-2} " == " ${CUDA_VER::-2} " ]]
        then
            CUDNN_VER=${CUDN_LIST[NV_NO]}
            NV_VER=${NV_DRIVER[NV_NO]}
            break
        fi
        let cnt++
    done
    echo "╔══╣ Install: NVIDIA-Driver $NV_VER & CUDA $CUDA_VER & cuDNN $CUDNN_VER in Ubuntu $UB_VER (STARTING) ╠══╗"
else
    echo "Please select one of the following CUDA versions"
    echo ${CUDA_LIST[*]}
    echo "For example: bash install_cuda.sh --11.6.2"
    exit
fi


mkdir cudnn
CUDNN_FILE=cudnn/cudnn-local-repo-ubuntu${UB_VER}-${CUDNN_VER}_1.0-1_amd64.deb
if [[ -f "$CUDNN_FILE" ]]; then
    echo "$CUDNN_FILE exists."
else
    echo "$CUDNN_FILE does not exist."
    echo "Please, download it from here: https://developer.nvidia.com/cudnn"
    echo "Then put it inside the cudnn folder!"
    exit
fi


# Removing previous CUDA Toolkit
sudo apt-get purge -y \
    "*cuda*" \
    "*cublas*" \
    "*cufft*" \
    "*cufile*" \
    "*curand*" \
    "*cusolver*" \
    "*cusparse*" \
    "*gds-tools*" \
    "*npp*" \
    "*nvjpeg*" \
    "nsight*" 

# Removing previous NVIDIA drivers"
# sudo dpkg -P $(dpkg -l | grep nvidia-driver | awk '{print $2}')

# sudo apt-get autoremove -y


# Install NVIDIA drivers based on CUDA version
# sudo apt-get update
# sudo apt-get install -y \
#     nvidia-driver-${NV_VER::-6} \
#     nvidia-dkms-${NV_VER::-6}
# sudo apt-get install -y \
#     nvidia-driver-515 \
#     nvidia-dkms-515


# Pre-installation Actions: install kernel headers
sudo apt-get install -y \
    linux-headers-$(uname -r)


# Remove Outdated Signing Key
sudo rm /etc/apt/sources.list.d/cuda-*
sudo rm /etc/apt/sources.list.d/cudnn-*
sudo rm /usr/share/keyrings/cuda-*
sudo rm /usr/share/keyrings/cudnn-*
sudo apt-key del 7fa2af80
sudo apt-key del 3bf863cc


# Enroll the new Signing Key
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${UB_VER}/x86_64/cuda-ubuntu${UB_VER}.pin
sudo mv cuda-ubuntu${UB_VER}.pin /etc/apt/preferences.d/cuda-repository-pin-600


# Install local CUDA repository
wget https://developer.download.nvidia.com/compute/cuda/${CUDA_VER}/local_installers/cuda-repo-ubuntu${UB_VER}-${CUDA_VER::-4}-${CUDA_VER:3:-2}-local_${CUDA_VER}-${NV_VER}-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu${UB_VER}-${CUDA_VER::-4}-${CUDA_VER:3:-2}-local_${CUDA_VER}-${NV_VER}-1_amd64.deb

if [[ "${CUDA_VER::-2}" > "11.6" ]]; then
    sudo cp /var/cuda-repo-ubuntu${UB_VER}-${CUDA_VER::-4}-${CUDA_VER:3:-2}-local/cuda-*-keyring.gpg /usr/share/keyrings/
elif [[ "${CUDA_VER::-2}" == "11.6" ]]; then
    sudo apt-key add /var/cuda-repo-ubuntu${UB_VER}-${CUDA_VER::-4}-${CUDA_VER:3:-2}-local/7fa2af80.pub
elif [[ "${CUDA_VER::-2}" == "11.3" ]]; then
    sudo apt-key add /var/cuda-repo-ubuntu${UB_VER}-${CUDA_VER::-4}-${CUDA_VER:3:-2}-local/7fa2af80.pub
else 
    exit
fi

rm cuda-repo-ubuntu${UB_VER}-${CUDA_VER::-4}-${CUDA_VER:3:-2}-local_${CUDA_VER}-${NV_VER}-1_amd64.deb

sudo apt-get update
sudo apt-get install -y \
    cuda-${CUDA_VER::-4}-${CUDA_VER:3:-2}
sudo apt-get install -y \
    nvidia-gds


# Post-installation Actions: Environment Setup
echo "export PATH=/usr/local/cuda-${CUDA_VER::-2}/bin\${PATH:+:\${PATH}}" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/cuda-${CUDA_VER::-2}/lib64\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}" >> ~/.bashrc


# Post-installation Actions: POWER9 Setup
sudo systemctl enable nvidia-persistenced

sudo cp /lib/udev/rules.d/40-vm-hotadd.rules /etc/udev/rules.d
sudo sed -i '/SUBSYSTEM=="memory", ACTION=="add"/d' /etc/udev/rules.d/40-vm-hotadd.rules


# Post-installation Actions: Install Third-party Libraries
sudo apt-get install -y \
    g++ \
    freeglut3-dev \
    build-essential \
    libx11-dev \
    libxmu-dev \
    libxi-dev \
    libglu1-mesa \
    libglu1-mesa-dev \
    libfreeimage-dev


# Dowload CUDA-Samples
git clone https://github.com/nvidia/cuda-samples
mv cuda-samples ~/

# Install cuDNN
# IMPORTANT: Still not supported for 11.7
sudo apt-get install -y \
    zlib1g

cd cudnn
sudo dpkg -i cudnn-local-repo-ubuntu${UB_VER}-${CUDNN_VER}_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-*/cudnn-local-*-keyring.gpg /usr/share/keyrings/
cd ..

sudo apt-get update
sudo apt-get install -y \
    libcudnn8=${CUDNN_VER}-1+cuda${CUDA_VER::-2}

sudo apt-get install -y \
    libcudnn8-dev=${CUDNN_VER}-1+cuda${CUDA_VER::-2}

sudo apt-get install -y \
    libcudnn8-samples=${CUDNN_VER}-1+cuda${CUDA_VER::-2}

sudo cp -r /usr/src/cudnn_samples_v8/ ~/


echo "╚══╣ Install: NVIDIA-Driver $NV_VER & CUDA $CUDA_VER & cuDNN $CUDNN_VER in Ubuntu $UB_VER (FINISHING) ╠══╝"

echo " Reboot now: $ reboot "
# sudo reboot
