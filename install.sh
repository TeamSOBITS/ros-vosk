#!/bin/bash


echo "╔══╣ Install: Speech Recognition Vosk for ROS (STARTING) ╠══╗"


# Keep the current directory
CURRENT_DIR=$(pwd)

# Install necessary packages
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-std-msgs

sudo apt-get install -y \
    python3-tk

# Install necessary packages from pip3
python3 -m pip install -U pip
python3 -m pip install \
    beautifulsoup4 \
    soundfile \
    playsound \
    vosk \
    sounddevice

# Install "sobits_msgs"
cd ..
git clone -b feature/humble-devel https://github.com/TeamSOBITS/sobits_msgs.git

cd $CURRENT_DIR


echo "╚══╣ Install: Speech Recognition Vosk for ROS (FINISHED) ╠══╝"
