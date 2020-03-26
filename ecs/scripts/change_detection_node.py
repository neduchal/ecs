#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

class ChangeDetectionNode:

    def __init__(self):
        self.node_name = rospy.get_name()

   


if __name__ == "__main__":
    rospy.init_node("ecs_change_detection_node")
    change_detection_node = ChangeDetectionNode()
    rospy.spin()
