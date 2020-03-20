#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import switcher_api
from std_msgs.msg import String
import cjson

class SwitcherNode:

    def __init__(self, setting_file):
        self.switch_sub = rospy.Subscriber("/switcher/switch", String, self.callback_switch)
        self.active_process = None
        self.switcher_list = []
        self.default_lf = None
        self.load_switcher_settings(setting_file)

    def run_process(self, id):
        for process in self.switcher_list:
            if (id == process[0]):
                self.stop_process()
                self.active_process = switcher_api.find_and_launch(lfs[1], lfs[2])        
                self.active_process = switcher_api.start_process(process[1], process[2])
                break   
        else:
            rospy.logwarn("Launch File %s Not Found", id)

    def stop_process(self):
        switcher_api.stop_launch(self.active_process)


    def load_switcher_settings(self, filename):
        settings = cjson.load(filename)
        self.switcher_list = settings["lfs"]
        self.default_lf = settings["default_lf"]
                

    def callback_switch(self, data):
        self.run_process(data.data)

if __name__ == "__main__":
    # TODO: Add ros params loading
    rospy.init_node("ecs_switcher_node")
    switcher_node = SwitcherNode("")
    rospy.spin()