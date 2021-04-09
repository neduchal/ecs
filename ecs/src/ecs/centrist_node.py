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
        self.cl = centrist.load()
        self.node_name = rospy.get_name()
        self.cv_bridge = CvBridge()
        self.spatial_division = rospy.get_param(
            self.node_name + "/spatial_division", default=1)
        self.desc_length = rospy.get_param(
            self.node_name + "/description_length", default=256)
        self.descriptor_service = rospy.Service(
            "/ecs/descriptor", Descriptor, self.handle_descriptor_service)
        pass

    def handle_descriptor_service(self, req):
        img = self.cv_bridge.imgmsg_to_cv2(req.img, desired_encoding="CV_8UC3")
        h = self.process_img(img)
        return DescriptorResponse(h)

    def process_single_channel(self, im_one_channel):
        centrist_im = centrist.centrist_im(self.cl, im_one_channel)
        return desc.spatial_histogram_bw(centrist_im, self.spatial_division, self.spatial_division, self.desc_length)

    def process_img(self, img):
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h = self.process_single_channel(img)
        return h


if __name__ == "__main__":
    rospy.init_node("ecs_centrist_descriptor")
    centrist_obj = CentristDescriptor()
    rospy.spin()
