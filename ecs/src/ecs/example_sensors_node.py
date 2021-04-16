#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import description as desc
from cv_bridge import CvBridge
from ecs.msg import Sensors, SensorValue 
import cv2


class SensorNode:

    def __init__(self):
        self.settings = None
        self.load_settings()
        self.node_name = rospy.get_name()
        self.cv_bridge = CvBridge()
        self.sensors_subscriber = rospy.Subscriber(self.settings.get(
            "input_topic"), Sensors, callback=self.sensors_callback, queue_size=10)
        self.sensors_publisher = rospy.Publisher(self.settings.get("sensors_topic"), SensorValue, queue_size=10)

    def load_settings(self):
        self.print_info(f"Example sensors node")
        self.settings = rospy.get_param("sensors_node")
        if self.settings.get("input_topic") is None:
            self.settings["input_topic"] = "/sensors"
        if self.settings.get("sensors_topic") is None:
            self.settings["sensors_topic"] = "/ecs/sensors"
        self.print_info(
            f"input topic is set to {self.settings['input_topic']}")            
        self.print_info(
            f"sensors topic is set to {self.settings['sensors_topic']}")

    def print_info(self, msg):
        rospy.loginfo(f"[{rospy.get_name()}]: {msg}")

    def sensors_callback(self, msg):
        temperature_msg = SensorValue()
        temperature_msg.sensor = "temperature_sensor"
        temperature_msg.value = msg.temperature
        self.sensors_publisher.publish(temperature_msg)
        humidity_msg = SensorValue()
        humidity_msg.sensor = "humidity_sensor"
        humidity_msg.value = msg.humidity
        self.sensors_publisher.publish(humidity_msg)
        distance_msg = SensorValue()
        distance_msg.sensor = "distance_sensor"
        distance_msg.value = msg.distance
        self.sensors_publisher.publish(distance_msg)

if __name__ == "__main__":
    rospy.init_node("ecs_example_sensors_topic")
    sensors_node_obj = SensorNode()
    rospy.spin()
