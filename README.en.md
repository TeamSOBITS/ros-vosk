<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

ROS VOSK
======================
<!--  TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#installation">installation</a>
    </li>
    <li>
    <a href="#launch">Launch and Usage</a>
    </li>
    <li>
    <a href="#Service Name">Service Name</a>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <li><a href="#Main Authors">Main Authors</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->

  </ol>
</details>


## Introduction

A ROS package for speech-to-text services based on [Vosk](https://github.com/alphacep/vosk-api) and [ros_vosk](https://github.com/alphacep/ros-vosk).

## Installation

1. Download this package

```bash
cd ~/catkin_ws/src/
git clone https://github.com/TeamSOBITS/ros_vosk
# git clone https://github.com/TeamSOBITS/speech_recognition_vosk
```

2. Install Dependencies

```bash
cd ros_vosk/
# cd speech_recognition_vosk/
bash install.sh

# or you can do it manually
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
# Install "sobits_msgs"
cd ../
git clone https://github.com/TeamSOBITS/sobits_msgs.git
```


3. Don't forget to run `catkin_make`
```bash
cm

# or
cd ~/catkin_ws/ && catkin_make
```

> **Warning**
> Sometimes the new generated msg might not be recognized and `import error` might appear. In such case, you will need to open a new terminal or restart your container (if you are using Docker).


## Launch

1. Check the model you want to use is properly selected in the config file.
```bash
cat ~/catkin_ws/src/ros_vosk/cfg/params.yaml
# cat ~/catkin_ws/src/speech_recognition_vosk/cfg/params.yaml
```

> **Note**
> Remember that you can use any language model from the [list of models compatible with Vosk-API](https://alphacephei.com/vosk/models).

> **Note**
> The model should be downloaded automatically if it exists in the database.

2. Launch the node

```bash
# Launch the speech recognition node
roslaunch ros_vosk ros_vosk.launch

# if you want to select language
# English ver
roslaunch ros_vosk ros_vosk.launch lang=en
# Japanese ver
roslaunch ros_vosk ros_vosk.launch lang=ja

# or by running
rosrun ros_vosk vosk_node.py
# rosrun speech_recognition_vosk vosk_node.py
```
3. First execution

The first run will bring up a screen similar to the one below to download the model.
![img1](img/image.png)  
If you want to use English, select “English” for “Select language” and “vosk-model-en-us-0.15” for “Select model” to download.

If you want to use Japanese, select “Japanese” for “Select language” and “vosk-model-ja-0.22” for “Select model” to download.

## Service Name
```
/speech_recognition  #same web_speech_recognition
```


## Interface

### Publishing Topics
- speech_recognition/vosk_result    -> vosk_node.py publishes a custom "speech_recognition" message
- speech_recognition/final_result   -> vosk_node.py publishes a simple string with the final result
- speech_recognition/partial_result -> vosk_node.py publishes a simple string with the partial result
- tts/status -> tts_engine.py publishes the state of the engine. True if it is speaking False if it is not. If the status is true vosk_node won't process the audio stream so it won't listen to itself 
- tts/phrase -> tts_engine.py subscribes to this topic in order to speak the given string. Name your desire and it shall be heard by all in the room..


<!-- MILESTONE -->
## Milestone

- [x] OSS
    - [x] Improved Documentation
    - [x] Update of customized msgs

See the [open issues][license-url] for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_msgs.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_msgs/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_msgs.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_msgs/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_msgs.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_msgs/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_msgs.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_msgs/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_msgs.svg?style=for-the-badge
[license-url]: LICENSE