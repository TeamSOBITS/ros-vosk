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
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
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

<!-- GETTING STARTED -->
## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ cd src/
   ```
2. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/ros_vosk
   ```
3. Navigate into the repository.
   ```sh
   $ cd ros_vosk/
   ```
4. Install the dependent packages.
   ```sh
   $ bash install.sh
   ```
5. Compile the package.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ catkin_make
   ```

> **Warning**
> Sometimes the new generated msg might not be recognized and `import error` might appear. In such case, you will need to open a new terminal or restart your container (if you are using Docker).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


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