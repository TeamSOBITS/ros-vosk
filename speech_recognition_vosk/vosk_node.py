#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic 

# This is a node that integrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listening to the audio stream so it won't listen to itself :)

# It publishes to the topic speech_recognition/vosk_result a custom "speech_recognition" message
# It publishes to the topic speech_recognition/final_result a simple string
# It publishes to the topic speech_recognition/partial_result a simple string

import os
import sys
import json
import queue
import time
import vosk
import sounddevice as sd
from mmap import MAP_SHARED
from playsound import playsound
import getpass


import rclpy
from rclpy.node import Node

# from speech_recognition_vosk.msg import Speech_recognition
from sobits_msgs.srv import SpeechRecognition
from std_msgs.msg import String, Bool

from . import vosk_model_downloader as downloader

class VoskSR(Node):
    def __init__(self):
        super().__init__('vosk_sr')

        self.declare_parameter('vosk.model', "vosk-model-small-en-us-0.15")
        self.declare_parameter('recognition_mode', "service")
        self.declare_parameter('vosk.sample_rate', 44100)
        self.declare_parameter('vosk.blocksize', 16000)

        model_name = self.get_parameter('vosk.model').get_parameter_value().string_value
        self.mode = self.get_parameter('recognition_mode').get_parameter_value().string_value

        self.package_path = "/home/" + str(getpass.getuser()) + "/colcon_ws/src/speech_recognition_vosk"
        
        models_dir = os.path.join(self.package_path, 'models')
        model_path = os.path.join(models_dir, model_name)
        
        if not os.path.exists(model_path):
            model_downloader = downloader.ModelDownloader()
            model_link, _ = model_downloader.get_model_link(model_name)

            if model_link is None:
                print(f"model '{model_name}' not found in '{models_dir}'! Please use the GUI to download it or configure an available model...")
                model_downloader.execute()
            else:
                print(f"model '{model_name}' not found in '{models_dir}'! Downloading model...")
                model_downloader.download(model_name)
                
            model_name = model_downloader.model_to_download
        
        self.get_parameter('vosk.model').get_parameter_value().string_value

        self.tts_status = False

        # ROS node initialization

        self.rate = self.create_rate(100)

        self.final_result = ""
        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            self.get_logger().fatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        
        self.samplerate = self.get_parameter('vosk.sample_rate').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('vosk.blocksize').get_parameter_value().integer_value

        self.model = vosk.Model(model_path)
        
        self.create_subscription(Bool, '/tts/status', self.tts_get_status, 10)

    def cleanup(self):
        self.get_logger().warn("Shutting down VOSK speech recognition node...")
    
    def stream_callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
        
    def tts_get_status(self, msg):
        self.tts_status = msg.data

    def speech_recognize(self, request, response):
        self.final_result = ""
        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=self.blocksize, device=self.input_dev_num, dtype='int16', channels=1, callback=self.stream_callback):

                if self.mode == "service":
                    playsound(os.path.join(self.package_path, 'mp3', 'start_sound.mp3'))
                    self.get_logger().info('Service Started')
                else:
                    self.get_logger().info('Voice Trigger Started')

                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                isRecognized = False
                isRecognized_partially = False

                current_time = time.time()
                start_time = current_time
                ts = []

                while rclpy.ok():
                    if self.tts_status:
                        with self.q.mutex:
                            self.q.queue.clear()
                        rec.Reset()
                    else:
                        data = self.q.get()
                        if rec.AcceptWaveform(data):
                            result = rec.FinalResult()
                            diction = json.loads(result)
                            lentext = len(diction["text"])

                            if lentext > 2:
                                result_text = diction["text"]
                                isRecognized = True
                            else:
                                isRecognized = False
                            rec.Reset()
                        else:
                            result_partial = rec.PartialResult()
                            if len(result_partial) > 20:
                                isRecognized_partially = True
                                partial_dict = json.loads(result_partial)
                                partial = partial_dict["partial"]

                        if isRecognized:
                            self.final_result = result_text
                            self.partial_result = "unk"
                            time.sleep(0.1)
                            if self.mode == "trigger":
                                self.pub_final.publish(String(data=result_text))
                            isRecognized = False

                        elif isRecognized_partially:
                            if partial != "unk":
                                self.final_result = "unk"
                                self.partial_result = partial
                                time.sleep(0.1)
                                partial = "unk"
                                isRecognized_partially = False
                                if self.mode == "service":
                                    if self.partial_result and self.partial_result != "unk":
                                        ts.append(self.partial_result)

                    if self.mode == "service":
                        current_time = time.time()
                        if current_time - start_time > request.timeout_sec:
                            break

                if self.mode == "service":
                    if self.final_result and self.final_result != "unk":
                        ts.append(self.final_result)
                    self.get_logger().info(str(ts))

                    playsound(os.path.join(self.package_path, 'mp3', 'end_sound.mp3'))

                    response.transcript = ts
                    return response

        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            self.get_logger().info("Stopping the VOSK speech recognition node...")
            time.sleep(1)
            print("node terminated")

    def wait_server(self):
        if self.mode == "service":
            self.get_logger().info("Waiting for service...")
            self.create_service(SpeechRecognition, '/speech_recognition', self.speech_recognize)
        elif self.mode == "trigger":
            self.pub_final = self.create_publisher(String, 'speech_recognition/result', 10)
            srv = SpeechRecognition.Request()
            srv.timeout_sec = 1000
            self.speech_recognize(srv, SpeechRecognition.Response())
        else:
            self.get_logger().fatal("Error of the mode select")

def main(args=None):
    rclpy.init(args=args)
    rec = VoskSR()
    rec.wait_server()
    rclpy.spin(rec)

    rec.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
