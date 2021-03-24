#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import description as desc
import leargist
from cv_bridge import CvBridge
from ecs.srv import Descriptor, DescriptorResponse
from PIL import Image as Image


class CentristDescriptor:

    def __init__(self):
        self.desc_length = 256
        self.cv_bridge = CvBridge()
        self.descriptor_topic = rospy.get_param(
            "/ecs/descriptor_topic", default="/ecs/descriptor")
        self.descriptor_service = rospy.Service(
            self.descriptor_topic, Descriptor, self.handle_descriptor_service)

    def handle_descriptor_service(self, req):
        img = self.cv_bridge.imgmsg_to_cv2(req.img, desired_encoding="CV_8UC3")
        h = self.process_img(img)
        return DescriptorResponse(h)

    def process_img(self, im):
        img = Image.fromarray(np.uint8(im)).resize((128, 128))
        desc_vector = leargist.color_gist(img)
        return desc_vector


if __name__ == "__main__":
    rospy.init_node("ecs_centrist_descriptor")
    centrist_obj = CentristDescriptor()
    rospy.spin()
