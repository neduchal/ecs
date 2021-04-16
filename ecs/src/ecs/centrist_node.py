#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import description as desc
import centrist
from cv_bridge import CvBridge
from ecs.srv import Descriptor, DescriptorResponse
import cv2


class CentristDescriptor:

    def __init__(self):
        self.settings = None
        self.load_settings()
        self.cl = centrist.load()
        self.node_name = rospy.get_name()
        self.cv_bridge = CvBridge()
        self.descriptor_service = rospy.Service(
            self.settings.get("descriptor_service"), Descriptor, self.handle_descriptor_service)
        pass

    def load_settings(self):
        self.print_info(f"Centrist descriptor node")
        self.settings = rospy.get_param("centrist_descriptor")
        if self.settings.get("spatial_division") is None:
            self.settings["spatial_division"] = 4
        if self.settings.get("descriptor_length") is None:
            self.settings["descriptor_length"] = 16
        if self.settings.get("descriptor_service") is None:
            self.settings["descriptor_service"] = "/ecs/descriptor"
        self.print_info(
            f"spatial division is set to {self.settings['spatial_division']}")
        self.print_info(
            f"descriptor length is set to {self.settings['descriptor_length']}")
        self.print_info(
            f"descriptor service is set to {self.settings['descriptor_service']}")

    def print_info(self, msg):
        rospy.loginfo(f"[{rospy.get_name()}]: {msg}")

    def handle_descriptor_service(self, req):
        img = self.cv_bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
        h = self.process_img(img)
        return DescriptorResponse(h)

    def process_single_channel(self, im_one_channel):
        sd = self.settings.get("spatial_division")
        desc_len = self.settings.get("descriptor_length")
        centrist_im = centrist.centrist_im(self.cl, im_one_channel)
        return desc.spatial_histogram_bw(centrist_im, sd, sd, desc_len)

    def process_img(self, img):
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h = self.process_single_channel(img).astype(int)
        return h.tolist()


if __name__ == "__main__":
    rospy.init_node("ecs_centrist_descriptor")
    centrist_obj = CentristDescriptor()
    rospy.spin()
