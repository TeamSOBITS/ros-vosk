#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
import re
from subprocess import Popen

class TopicSubscriber():

    def __init__(self):
        
        self.sub_msg = rospy.Subscriber('/speech_recognition/final_result', String, self.callback_message )
        self.pub = rospy.Publisher("/voice_trigger/start_flag", Bool, queue_size=1)
        self.trigger = False
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.trigger)
            if self.trigger:
                last_time_s = time.time()
                while not rospy.is_shutdown():
                    self.pub.publish(self.trigger)
                    if ((time.time() - last_time_s) > 3):
                        self.pub.publish(self.trigger)
                        break
                    r.sleep()
                break
            r.sleep()
        Popen(['rosnode', 'kill', '/vosk_engine'])
        Popen(['rosnode', 'kill', '/tts_engine'])
        self.sub_msg.unregister()



    def callback_message(self, msg):
        s_msg = str(msg)

        en_msg = s_msg.encode()
        de_msg = en_msg.decode("unicode-escape")
        listen_word = re.findall('"(.*?)"', de_msg)

        word_list = rospy.get_param("trigger_word").split()

        print()
        print("======================================================")
        print("---------------------voice_word-----------------------")
        print(de_msg)
        print("------------------------------------------------------")
        print()
        print("--------------------trigger_word---------------------")
        print(word_list)
        print("------------------------------------------------------")
        print("======================================================")
        print()

        and_list = set(word_list) & set(listen_word)
        word = set(word_list)
        listen1 = "".join(listen_word)
        listen2 = listen1.split() 
        listen = set(listen2)

        if word & listen:
            print("listener_wordが検出されたのでvoskを終了します")
            rospy.loginfo("*** publish Trueを送りました ***")
            self.trigger = True  


if __name__ == '__main__':
    
    rospy.init_node('voice_trigger')
    
    node = TopicSubscriber()

    rospy.spin()
