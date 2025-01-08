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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

# from speech_recognition_vosk.msg import Speech_recognition
from sobits_interfaces.action import SpeechRecognition
from std_msgs.msg import String, Bool

from ament_index_python.packages import get_package_share_directory

# from . import model_downloader as downloader

class VoskSR(Node):
    def __init__(self):
        super().__init__('vosk_node')

        self.declare_parameter('model', "vosk-model-small-en-us-0.15")
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('blocksize', 16000)

        self.model_path = self.get_parameter('model').get_parameter_value().string_value
        self.samplerate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('blocksize').get_parameter_value().integer_value

        self.sound_folder_path = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')

        if not os.path.exists(self.model_path):
            self.get_logger().fatal("\033[31m[NOT MODEL] " + str(self.model_path) + "\033[0m")
            return

        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            self.get_logger().fatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        

        self.model = vosk.Model(self.model_path)
        

        self.get_logger().info("Waiting for service...")
        self.server = ActionServer(
            self,
            SpeechRecognition,
            "speech_recognition",
            execute_callback=self.speech_recognize,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def stream_callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    async def speech_recognize(self, goal_handle):
        feedback = SpeechRecognition.Feedback()
        response = SpeechRecognition.Result()

        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=self.blocksize, device=self.input_dev_num, dtype='int16', channels=1, callback=self.stream_callback):

                if (not goal_handle.request.silent_mode):
                    playsound(os.path.join(self.sound_folder_path, 'start_sound.mp3'))
                self.get_logger().info('Server Start')

                rec = vosk.KaldiRecognizer(self.model, self.samplerate)

                last_wip_text_to_feedback = ""
                feedback.addition_text = ""
                response.result_text = ""

                start_time = time.time()
                last_feedback_time = time.time()
                while rclpy.ok():

                    partial = None
                    result_text = None
                    isRecognized = False
                    isRecognized_partially = False

                    if goal_handle.is_cancel_requested:
                        self.get_logger().info('Goal canceled')
                        self.get_logger().info('Wip Result : ' + response.result_text)
                        goal_handle.canceled()
                        return response

                    data = self.q.get()
                    if rec.AcceptWaveform(data):

                        result = rec.FinalResult()
                        diction = json.loads(result)
                        lentext = len(diction["text"])

                        if lentext > 0:
                            result_text = diction["text"]
                            isRecognized = True

                        rec.Reset()
                    else:

                        result_partial = rec.PartialResult()
                        if len(result_partial) > 20:  # there is partial

                            isRecognized_partially = True
                            partial_dict = json.loads(result_partial)
                            partial = partial_dict["partial"]


                    if (isRecognized or (isRecognized_partially and partial)): time.sleep(0.1)

                    if (result_text):
                        if (len(response.result_text) != 0):
                            response.result_text += " "
                        response.result_text += result_text

                    if ((time.time() - last_feedback_time) > (1.0/float(goal_handle.request.feedback_rate))):
                        if (partial):
                            wip_result = response.result_text + partial
                        else:
                            wip_result = response.result_text

                        feedback.addition_text = wip_result[len(last_wip_text_to_feedback):]
                        goal_handle.publish_feedback(feedback)

                        last_wip_text_to_feedback = response.result_text
                        last_feedback_time = time.time()

                    if ((time.time() - start_time) > goal_handle.request.timeout_sec):
                        if (partial):
                            response.result_text += partial
                        self.get_logger().info("Result : " + response.result_text)
                        break

                if (not goal_handle.request.silent_mode):
                    playsound(os.path.join(self.sound_folder_path, 'end_sound.mp3'))

                goal_handle.succeed()
                return response

        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            self.get_logger().info("Stopping the VOSK speech recognition node...")
            time.sleep(1)
            print("node terminated")



def main(args=None):
    try:
        rclpy.init(args=args)

        rec = VoskSR()

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()

        rclpy.spin(rec, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
if __name__ == '__main__':
    main()
