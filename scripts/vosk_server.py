#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic 

# This is a node that intergrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listenning to the audio stream so it won't listen to it self :)

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

import rospy
import rospkg
# from speech_recognition_vosk.msg import Speech_recognition
from sobits_msgs.srv import Speech_Recognition
from std_msgs.msg import String, Bool

import vosk_ros_model_downloader as downloader

class vosk_sr():
    def __init__(self):
        model_name = rospy.get_param('vosk_model', "vosk-model-small-en-us-0.15")

        rospack = rospkg.RosPack()
        rospack.list()
        self.package_path = rospack.get_path('speech_recognition_vosk')
        
        models_dir = os.path.join(self.package_path, 'models')
        model_path = os.path.join(models_dir, model_name)
        
        if not os.path.exists(model_path):
            model_downloader = downloader.model_downloader()
            model_link, _ = model_downloader.get_model_link(model_name)
            # rospy.loginfo("model link: %s", model_link)

            if model_link == None:
                print (f"model '{model_name}' not found in '{models_dir}'! Please use the GUI to download it or configure an available model...")
                model_downloader.execute()
            else:
                print (f"model '{model_name}' not found in '{models_dir}'! Downloading model...")
                model_downloader.download(model_name)
                
            model_name = model_downloader.model_to_download
        
        if not rospy.has_param('vosk_model'):
            rospy.set_param('vosk_model', model_name)

        self.tts_status = False

        # ROS node initialization
        
        # self.pub_vosk = rospy.Publisher('speech_recognition/vosk_result',Speech_recognition, queue_size=10)
        # self.pub_final = rospy.Publisher('speech_recognition/final_result',String, queue_size=10)
        # self.pub_partial = rospy.Publisher('speech_recognition/partial_result',String, queue_size=10)

        self.rate = rospy.Rate(100)

        rospy.on_shutdown(self.cleanup)

        # self.msg = Speech_recognition()
        # self.msg = String()
        self.final_result = ""
        self.partial_result = ""

        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            rospy.logfatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:
        
        # self.samplerate = int(device_info['default_samplerate'])
        self.samplerate = rospy.get_param("/vosk_samplerate", 16000)
        self.blocksize = rospy.get_param("/vosk_blocksize", 16000)
        # rospy.set_param('vosk/sample_rate', self.samplerate)

        self.model = vosk.Model(model_path)

        #TODO GPUInit automatically selects a CUDA device and allows multithreading.
        # gpu = vosk.GpuInit() #TODO

    
    def cleanup(self):
        rospy.logwarn("Shutting down VOSK speech recognition node...")
    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
        
    def tts_get_status(self,msg):
        self.tts_status = msg.data

    def tts_status_listenner(self):
        rospy.Subscriber('/tts/status', Bool, self.tts_get_status)

    def speech_recognize(self, srv):
        # rospy.loginfo("start service")
        self.final_result = ""
        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16',
                               channels=1, callback=self.stream_callback):
                playsound(os.path.join(self.package_path, 'mp3', 'start_sound.mp3'))

                rospy.loginfo('Service Started')
                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                isRecognized = False
                isRecognized_partially = False

                current_time = time.time()
                start_time   = current_time
                ts = []

                while not rospy.is_shutdown():
                    self.tts_status_listenner()

                    if self.tts_status == True:
                        # If the text to speech is operating, clear the queue
                        with self.q.mutex:
                            self.q.queue.clear()
                        rec.Reset()

                    elif self.tts_status == False:
                        data = self.q.get()
                        if rec.AcceptWaveform(data):
                            # In case of final result
                            result = rec.FinalResult()

                            diction = json.loads(result)
                            lentext = len(diction["text"])

                            if lentext > 2:
                                result_text = diction["text"]
                                isRecognized = True
                            else:
                                isRecognized = False
                            # Resets current results so the recognition can continue from scratch
                            rec.Reset()
                        else:
                            # In case of partial result
                            result_partial = rec.PartialResult()
                            if (len(result_partial) > 20):

                                isRecognized_partially = True
                                partial_dict = json.loads(result_partial)
                                partial = partial_dict["partial"]

                        if (isRecognized is True):
                            # self.msg.isSpeech_recognized = True
                            # self.msg.time_recognized = rospy.Time.now()
                            self.final_result = result_text
                            self.partial_result = "unk"
                            rospy.sleep(0.1)
                            isRecognized = False

                        elif (isRecognized_partially is True):
                            if partial != "unk":
                                # self.msg.isSpeech_recognized = False
                                # self.msg.time_recognized = rospy.Time.now()
                                self.final_result = "unk"
                                self.partial_result = partial
                                rospy.sleep(0.1)
                                partial = "unk"
                                isRecognized_partially = False
                                if ((self.partial_result != "") and (self.partial_result != "unk")):
                                    ts += [self.partial_result]

                    current_time = time.time()
                    if ((current_time - start_time) > srv.timeout_sec):
                        break

                # rospy.loginfo(self.final_result)
                if ((self.final_result != "") and (self.final_result != "unk")):
                    ts += [self.final_result]
                rospy.loginfo(ts)

                playsound(os.path.join(self.package_path, 'mp3', 'end_sound.mp3'))

                return SpeechRecognitionVoskResponse(transcript=ts)
            
        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            rospy.loginfo("Stopping the VOSK speech recognition node...")
            rospy.sleep(1)
            print("node terminated")

    def wait_server(self):
        rospy.loginfo("Waiting for service...")
        rospy.Service('/speech_recognition', SpeechRecognitionVosk, self.speech_recognize)
        # rospy.spin()



if __name__ == '__main__':
    try:
        rospy.init_node('vosk', anonymous=False)
        rec = vosk_sr()
        rec.wait_server()
        rospy.spin()

    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the vosk speech recognition node...")
        rospy.sleep(1)
        print("node terminated")