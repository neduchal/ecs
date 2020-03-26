#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import switcher_api
from std_msgs.msg import String
import cjson


class SwitcherNode:

    def __init__(self):

        self.node_name = rospy.get_name()
        self.settings_file = rospy.get_param(
            self.node_name + "/settings_file", "")
        if self.settings_file == "":
            quit(0)
        self.switcher_sub_topic = rospy.get_param(
            self.node_name + "/switcher_sub_topic", "/ecs/switcher/switch")
        self.switch_sub = rospy.Subscriber(
            self.switcher_sub_topic, String, self.callback_switch)
        self.active_process = None
        self.active_process_type = None
        self.switcher_list = []
        self.default_process = None
        self.load_switcher_settings(self.settings_file)

    def run_process(self, id):
        for process in self.switcher_list:
            if (id == process["name"]):
                self.stop_process()
                self.active_process, self.active_process_type = switcher_api.start_process(
                    process["pkg"], process["process"])
                break
        else:
            rospy.logwarn("Process %s Not Found", id)

    def stop_process(self):
        switcher_api.stop_process(
            self.active_process, self.active_process_type)

    def load_switcher_settings(self, filename):
        if (not rospy.has_param("ecs_switcher")) or (not rospy.has_param("ecs_switcher_default_process")):
            rospy.signal_shutdown("ECS Switcher parameters not found!")
            exit(1)
        self.switcher_list = rospy.get_param("ecs_switcher")
        self.default_process == rospy.get_param("ecs_switcher_default_process")

    def callback_switch(self, data):
        self.run_process(data.data)


if __name__ == "__main__":
    rospy.init_node("ecs_switcher_node")
    switcher_node = SwitcherNode()
    rospy.spin()
