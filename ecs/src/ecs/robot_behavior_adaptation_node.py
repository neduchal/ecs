#!/usr/bin/env python
# -*- coding: utf-8 -*-

from typing import ItemsView
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import switcher_api


class RBANode:

    def __init__(self):

        #self.node_name = rospy.get_name()
        self.settings = None
        self.behaviors = []
        self.active_behavior_id = None
        self.load_settings()
        self.running_processes = []
        self.decision_subscriber = rospy.Subscriber(
            self.settings["input_topic"], String, self.decision_callback)

    def load_settings(self):
        self.settings = rospy.get_param("ecs_rba")
        self.behaviors = self.settings["behaviors"]
        self.default_behavior = [item["id"]
                                 for item in self.behaviors if "default" in item.keys()][0]

    def decision_callback(self, msg):
        self.active_behavior = msg.data

    def set_command(self, cmd):
        rospy.set_param(cmd[1], cmd[2])
        rospy.loginfo(f"Parameter {cmd[1]} set to value {cmd[2]}")        

    def stop_command(self, cmd):
        for item in self.running_processes:
            if item[0] == cmd[1]:
                switcher_api.stop_process(item[2], item[1])
                del item
                return True
        rospy.logwarn(
            f"Process {cmd[1]} is not running. It can not be stopped.")
        return False

    def run_command(self, cmd):
        process, process_type = switcher_api.start_process(cmd[1])
        self.running_processes.append((cmd[1], process_type, process))

    def set_active_robot_behavior(self, id):
        commands = []
        for item in self.behaviors:
            if item.get("id") == id:
                self.active_behavior_id = item.get("id")
                commands = item.get("commands")
                break
        else:
            return False
        for cmd_string in commands:
            cmd = cmd_string.split(",")
            if cmd[0] == "set":
                self.set_command()
            elif cmd[0] == "stop":
                self.stop_command(cmd)
            elif cmd[0] == "run":
                self.run_command(cmd)
            elif cmd[0] == "echo":
                print(cmd[1])
            else:
                rospy.logwarn(f"Unknown RBA command type:{cmd[1]}")
        return True


if __name__ == "__main__":
    rospy.init_node("ecs_rba_node")
    rba_node = RBANode()
    rospy.spin()
