#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

class ChangeDetectorNode:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.settings = None
        self.load_settings()

    def load_settings(self):
        if not rospy.has_param("ecs_change_detector"):
            rospy.signal_shutdown("ECS Change detector parameters not found")
        self.settings = rospy.get_param("ecs_change_detector")


if __name__ == "__main__":
    rospy.init_node("ecs_change_detector_node")
    change_detection_node = ChangeDetectorNode()
    rospy.spin()
