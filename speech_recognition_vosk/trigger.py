#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import re
from subprocess import Popen

class TopicSubscriber(Node):

    def __init__(self):
        super().__init__('voice_trigger')

        self.sub_msg = self.create_subscription(String, '/speech_recognition/result', self.callback_message, 10)
        self.pub = self.create_publisher(Bool, '/speech_recognition/trigger_flag', 10)
        self.trigger = False
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_time_s = None

    def timer_callback(self):
        if not self.trigger:
            self.pub.publish(Bool(data=self.trigger))
        else:
            if self.last_time_s is None:
                self.last_time_s = self.get_clock().now().seconds_nanoseconds()[0]
            else:
                self.pub.publish(Bool(data=self.trigger))
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                if (current_time - self.last_time_s) > 3:
                    self.pub.publish(Bool(data=self.trigger))
                    Popen(['ros2', 'node', 'kill', '/vosk_engine'])
                    Popen(['ros2', 'node', 'kill', '/tts_engine'])
                    self.destroy_node()
                    rclpy.shutdown()

    def callback_message(self, msg):
        s_msg = msg.data

        en_msg = s_msg.encode()
        de_msg = en_msg.decode("unicode-escape")
        listen_word = de_msg.split()

        word_list = ["stop"]

        self.get_logger().info("======================================================")
        self.get_logger().info("---------------------voice_word-----------------------")
        self.get_logger().info(de_msg)
        self.get_logger().info("------------------------------------------------------")
        self.get_logger().info("--------------------trigger_word---------------------")
        self.get_logger().info(str(word_list))
        self.get_logger().info("------------------------------------------------------")
        self.get_logger().info("======================================================")

        # and_list = set(word_list) & set(listen_word)
        word = set(word_list)
        listen = set(listen_word)

        if word & listen:
            self.get_logger().info("listener_wordが検出されたのでvoskを終了します")
            self.get_logger().info("*** publish Trueを送りました ***")
            self.trigger = True  


def main(args=None):
    rclpy.init(args=args)
    node = TopicSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
