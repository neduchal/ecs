#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np 
import description as desc
import centrist
from cv_bridge import CvBridge
from ecs.srv import Descriptor, DescriptorResponse

class CentristDescriptor:

    def __init__(self):
        self.cl = centrist.load()
        self.desc_length = 256
        self.cv_bridge = CvBridge()
        self.descriptor_service = rospy.Service("/ecs/descriptor", Descriptor, self.handle_descriptor_service)
        pass

    def handle_descriptor_service(self, req):
        img = self.cv_bridge.imgmsg_to_cv2(req.img, desired_encoding="CV_8UC3")
        h = self.process_img(img)
        return DescriptorResponse(h)

    def process_single_channel(self, im_one_channel):
        centrist_im = centrist.centrist_im(self.cl, im_one_channel)
        return desc.spatial_histogram_bw(centrist_im, 1, 1, self.desc_length)

    def process_img(self, im):
        h1 = self.process_single_channel(im[:,:,0])
        h2 = self.process_single_channel(im[:,:,1])
        h3 = self.process_single_channel(im[:,:,2])
        return np.concatenate((h1, h2, h3))

if __name__ == "__main__":
    rospy.init_node("ecs_centrist_descriptor")
    centrist_obj = CentristDescriptor()
    rospy.spin()