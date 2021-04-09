#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import switcher_api


class RBANode:

    """
        Robot Behaviour Adaptation Node Class

        It is supposed to react on the deecision of the Image Based Classification Node
        by running predefined (in config file) set of commands.
    """

    def __init__(self):
        self.settings = None
        self.behaviors = []
        self.active_behavior = None
        self.load_settings()
        self.running_processes = []
        self.decision_subscriber = rospy.Subscriber(
            self.settings["decision_topic"], String, self.decision_callback)
        self.print_info(f"Decision topic: {self.settings['decision_topic']}")            
        for behavior in self.behaviors:
            if behavior.get("default") == 1:
                self.set_active_robot_behavior(behavior.get("name"))


    def print_info(self, msg):
        rospy.loginfo(f"[{rospy.get_name()}]: {msg}")

    def load_settings(self):
        self.settings = rospy.get_param("ecs_rba")
        self.behaviors = self.settings["behaviors"]

    def decision_callback(self, msg):
        self.set_active_robot_behavior(msg.data)

    def set_command(self, cmd):
        rospy.set_param(cmd[1], cmd[2])
        self.print_info(f"Parameter {cmd[1]} set to value {cmd[2]}")

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

    def set_active_robot_behavior(self, name):
        commands = []
        for item in self.behaviors:
            if item.get("name") == name:
                self.active_behavior = item.get("name")
                commands = item.get("commands")
                break
        else:
            return False
        for cmd_string in commands:
            cmd = cmd_string.replace(" ","").split(",")
            if cmd[0] == "set":
                self.set_command(cmd)
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
