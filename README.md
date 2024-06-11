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
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#introduction">概要</a>
    </li>
    <li>
      <a href="#installation">iセットアップ</a>
    </li>
    <li>
    <a href="#launch">Launch</a>
    </li>
    <li>
    <a href="#Service Name">Service名</a>
    </li>
    <li><a href="#milestone">マイルストーン</a></li>
    <li><a href="#Main Authors">Main Authors</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->

  </ol>
</details>


## 概要
これは、[Vosk](https://github.com/alphacep/vosk-api)と[ros_vosk](https://github.com/alphacep/ros-vosk)に基づく音声テキストサービス用のROSパッケージになります。

## Installation

1. このパッケージをダウンロード

```bash
cd ~/catkin_ws/src/
git clone https://github.com/TeamSOBITS/ros_vosk
# git clone https://github.com/TeamSOBITS/speech_recognition_vosk
```

2. 依存関係のインストール

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


3. `catkin_make`を忘れないで！
```bash
cm

# or
cd ~/catkin_ws/ && catkin_make
```

> **Warning**
> 新しい生成された msg が認識されず、`import error` が表示されることがあります。そのような場合は、新しいターミナルを開くか、コンテナを再起動する必要があります（Dockerを使用している場合）


## Launch

1. 使用したいモデルがコンフィグファイルで正しく選択されていることを確認してください。
```bash
cat ~/catkin_ws/src/ros_vosk/cfg/params.yaml
# cat ~/catkin_ws/src/speech_recognition_vosk/cfg/params.yaml
```

> **Note**
> [list of models compatible with Vosk-API](https://alphacephei.com/vosk/models).の言語モデルを使用できることを忘れないでください。

> **Note**
> モデルがデータベースに存在すれば、自動的にダウンロードされるはずです。

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
3. 初めてlaunchを使用する場合

最初の実行では、モデルをダウンロードするために以下のような画面が表示されます。
![img1](img/image.png)  
英語を使用する場合は「Select language」で「English」を、「Select model」で「vosk-model-ja-0.15」を選択してダウンロードしてください。

日本語を使用する場合は、「言語の選択」で「日本語」を選択し、「モデルの選択」で「vosk-model-ja-0.22」を選択してダウンロードしてください。

## Service名
```
/speech_recognition  #same web_speech_recognition
```


## インターフェース

### Publishing Topics
- speech_recognition/vosk_result    -> vosk_node.py publishes a custom "speech_recognition" message
- speech_recognition/final_result   -> vosk_node.py publishes a simple string with the final result
- speech_recognition/partial_result -> vosk_node.py publishes a simple string with the partial result
- tts/status -> tts_engine.py publishes the state of the engine. True if it is speaking False if it is not. If the status is true vosk_node won't process the audio stream so it won't listen to itself 
- tts/phrase -> tts_engine.py subscribes to this topic in order to speak the given string. Name your desire and it shall be heard by all in the room..


<!-- MILESTONE -->
## マイルストーン

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