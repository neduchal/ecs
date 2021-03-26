#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from joblib import load
from sklearn import svm
from std_msgs.msg import Empty, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ecs.srv import Descriptor


class ImageBasedEnvironmentClassification:

    def __init__(self):
        # load parameters
        self.classifier_file = rospy.get_param(
            "/ecs/classifier_classic", default="default.joblib")
        self.image_topic = rospy.get_param(
            "/ecs/image_topic", default="/camera/image_raw")
        self.trigger_topic = rospy.get_param(
            "/ecs/trigger_topic", default="/ecs/trigger")
        self.decision_topic = rospy.get_param(
            "/ecs/decision_topic", default="/ecs/decision")
        self.descriptor_servicename = rospy.get_param(
            "/ecs/descriptor_service", default="/ecs/descriptor")
        self.classifier = load(self.classifier_file)  # joblib load
        self.image_subscriber = rospy.Subscriber(
            self.image_topic, Image, callback=self.image_subscriber_callback, queue_size=1)
        self.trigger_subscriber = rospy.Subscriber(
            self.trigger_topic, Empty, callback=self.trigger_callback, queue_size=1)
        self.decision_publisher = rospy.Publisher(
            self.decision_topic, Int8, queue_size=10)
        self.img = None
        self.cv_bridge = CvBridge()

    def image_subscriber_callback(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="CV_8UC3")

    def descriptor_service_client(self):
        rospy.wait_for_service(self.descriptor_servicename)
        try:
            descriptor_service = rospy.ServiceProxy(
                self.descriptor_servicename, Descriptor)
            resp1 = descriptor_service(
                self.cv_bridge.cv2_to_imgmsg(self.img, encoding="CV_8UC3"))
            return resp1.data
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def trigger_callback(self, msg):
        self.process()

    def process(self):
        desc_vector = self.descriptor_service_client()
        prediction = self.classifier.predict(desc_vector.reshape(1, -1))[0]
        self.decision_publisher.publish(prediction)


if __name__ == "__main__":

    rospy.init_node("ecs_classification_classic_node")
    ibec = ImageBasedEnvironmentClassification()
    rospy.spin()
