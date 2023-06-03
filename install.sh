#!/bin/bash


echo "╔══╣ Install: VOSK for ROS (STARTING) ╠══╗"

# Install necessary packages from debian
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-yaml \
    espeak

# Install necessary packages from pip3
python3 -m pip install \
    sounddevice \
    vosk \
    pyttsx3 \
    lxml

echo "╚══╣ Install: VOSK for ROS (FINISHED) ╠══╝"
