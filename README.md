Speech Recognition VOSK
======================

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
# roslaunch speech_recognition_vosk stt_vosk.launch

# or by running
rosrun ros_vosk vosk_node.py
# rosrun speech_recognition_vosk vosk_node.py
```

## Interface

### Publishing Topics
- speech_recognition/vosk_result    -> vosk_node.py publishes a custom "speech_recognition" message
- speech_recognition/final_result   -> vosk_node.py publishes a simple string with the final result
- speech_recognition/partial_result -> vosk_node.py publishes a simple string with the partial result
- tts/status -> tts_engine.py publishes the state of the engine. True if it is speaking False if it is not. If the status is true vosk_node won't process the audio stream so it won't listen to itself 
- tts/phrase -> tts_engine.py subscribes to this topic in order to speak the given string. Name your desire and it shall be heard by all in the room..

## Main Authors
- Angelo Antikatzidis <an.antikatzidis@gmail.com>
- Nickolay V. Shmyrev <nshmyrev@gmail.com>

## Maintainers
- Valentin Keith
- Okuma Yuki
- Yamada Ren
- Ono Fumiya
