#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import switcher_api
from std_msgs.msg import String


class SwitcherNode:

    def __init__(self):

        #self.node_name = rospy.get_name()
        self.settings = None
        self.processes = []
        self.default_process = None
        self.load_switcher_settings()
        self.switch_sub = rospy.Subscriber(
            self.settings["input_topic"], String, self.callback_switch)
        self.active_process = None
        self.active_process_type = None

    def run_process(self, id):
        for process in self.processes:
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

    def load_switcher_settings(self):
        if not rospy.has_param("ecs_switcher"):
            rospy.signal_shutdown("ECS Switcher parameters not found!")
            exit(1)
        self.settings = rospy.get_param("ecs_switcher")
        self.processes = rospy.get_param("ecs_switcher")["processes"]
        self.default_process = [item["name"]
                                for item in self.processes if "default" in item.keys()][0]

    def callback_switch(self, data):
        self.run_process(data.data)


if __name__ == "__main__":
    rospy.init_node("ecs_switcher_node")
    switcher_node = SwitcherNode()
    rospy.spin()
