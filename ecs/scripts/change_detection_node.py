#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from ecs.msg import SensorValue, EnvValue
import sensor

class DataAcquisition:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.settings = None
        self.sensors = []
        self.sensor_names = []
        self.load_settings()
        self.data_subscriber = rospy.Subscriber(
            self.settings["input_topic"], SensorValue, self.sensor_callback, queue_size=100)
        ecs_map_topic = rospy.get_param("ecs_map/input_topic")
        self.republisher = rospy.Publisher(
            ecs_map_topic, EnvValue, queue_size=10)

    def sensor_callback(self, msg):
        if not msg.sensor in self.sensor_names:
            rospy.logwarn(
                "Sensor %s is not subscribed by the node", msg.sensor)
        sensor = [item for item in self.sensors if item["name"] == msg.sensor]
        if "republish" in sensor.keys():
            env_msg = EnvValue()
            env_msg.layer = sensor["name"]
            env_msg.value = msg.value
            self.republisher.publish(env_msg)
        # TODO: Create sensors objects and process data
        # TODO: trigger
        pass

    def load_settings(self):
        if not rospy.has_param("ecs_data_acquisition"):
            rospy.signal_shutdown("ECS Data acquisition parameters not found")
        self.settings = rospy.get_param("ecs_data_acquisition")
        self.sensors = self.settings["sensors"]
        self.sensor_names = [item["name"] for item in self.sensors]

    def set_sensors(self):
        pass


if __name__ == "__main__":
    rospy.init_node("ecs_data_acquisition_node")
    da_node = DataAcquisition()
    rospy.spin()
