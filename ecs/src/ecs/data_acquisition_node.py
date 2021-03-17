#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rospy.core import rospyerr
import rospy
from ecs.msg import SensorValue, EnvValue
from std_msgs.msg import Empty
import cpd

class DataAcquisition:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.settings = None
        self.sensors = []
        self.sensor_names = []
        self.load_settings()
        self.data_subscriber = rospy.Subscriber(
            self.settings["input_topic"], SensorValue, self.sensor_callback, queue_size=100)
        ecs_map_topic = rospy.get_param("ecs_map/input_topic", default="/map")
        self.republisher = rospy.Publisher(
            ecs_map_topic, EnvValue, queue_size=10)
        self.trigger_publisher = rospy.Publisher("/ecs/trigger", Empty, queue_size=10)

    def sensor_callback(self, msg):
        if not msg.sensor in self.sensor_names:
            rospy.logwarn(
                "Sensor %s is not subscribed by the node", msg.sensor)
        sensor = [item for item in self.sensors if item["name"] == msg.sensor]
        if sensor.get("republish") == 1:
            env_msg = EnvValue()
            env_msg.layer = sensor["name"]
            env_msg.value = msg.value
            self.republisher.publish(env_msg)
        sensor["detector"].addValue(msg.value)
        if sensor["detector"].isChangePoint():
            trigger = Empty()
            self.trigger_publisher.publish(trigger)

    def load_settings(self):
        if not rospy.has_param("ecs_change_detector"):
            rospy.signal_shutdown("ECS Change detector parameters not found")
        self.settings = rospy.get_param("ecs_data_acquisition")
        self.sensors = self.settings["sensors"]
        self.sensor_names = [item["name"] for item in self.sensors]
        for sensor in self.sensors:
            par  = sensor["parameters"]
            if sensor["method"] == "diffratio":
                sensor["detector"] = cpd.DiffRatio(par["sensitivity_threshold"], par["min_length"])
            elif sensor["method"] == "varratio":          
                sensor["detector"] = cpd.VarRatio(par["var_stable"], par["sensitivity_threshold"], par["window_length"])
            else:
                rospyerr("Unknown detector")

if __name__ == "__main__":
    rospy.init_node("ecs_data_acquisition_node")
    da_node = DataAcquisition()
    rospy.spin()
