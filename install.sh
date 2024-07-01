#!/bin/bash


echo "╔══╣ Install: Speech Recognition Vosk for ROS (STARTING) ╠══╗"

# Install necessary packages from debian
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-yaml \
    portaudio19-dev \
    espeak


# Install necessary packages from pip3
python3 -m pip install \
    sounddevice \
    vosk \
    pyttsx3 \
    pyaudio \
    beautifulsoup4 \
    lxml \
    playsound

# Install "sobits_msgs"
cd ~/catkin_ws/src/
git clone https://github.com/TeamSOBITS/sobits_msgs.git
cd ~/catkin_ws/src/speech_recognition_vosk/

echo "╚══╣ Install: Speech Recognition Vosk for ROS (FINISHED) ╠══╝"
