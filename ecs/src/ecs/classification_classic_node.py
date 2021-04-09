#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import numpy as np
from joblib import load
from sklearn import svm
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ecs.srv import Descriptor


class ImageBasedEnvironmentClassification:

    def __init__(self):
        # load parameters
        self.settings = None
        self.load_settings()
        rospack = rospkg.RosPack()
        self.classifier_path = os.path.join(rospack.get_path(
            self.settings["classifier_pkg"]), self.settings["classifier_file"])
        self.image_subscriber = rospy.Subscriber(
            self.settings["camera_topic"], Image, callback=self.image_subscriber_callback, queue_size=1)
        self.trigger_subscriber = rospy.Subscriber(
            self.settings["trigger_topic"], Empty, callback=self.trigger_callback, queue_size=1)
        self.decision_publisher = rospy.Publisher(
            self.settings["decision_topic"], String, queue_size=10)
        self.print_info(f"Loaded classifier: {self.classifier_path}")
        self.print_info(f"Camera topic: {self.settings['camera_topic']}")
        self.print_info(f"Trigger topic: {self.settings['trigger_topic']}")
        self.print_info(f"Decision topic: {self.settings['decision_topic']}")
        self.img = None
        self.cv_bridge = CvBridge()
        self.classifier = load(self.classifier_path)

    def print_info(self, msg):
        rospy.loginfo(f"[{rospy.get_name()}]: {msg}")

    def load_settings(self):
        self.settings = rospy.get_param("ecs_ibec")

    def image_subscriber_callback(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="CV_8UC3")

    def descriptor_service_client(self):
        rospy.wait_for_service(self.settings["descriptor_service"])
        try:
            descriptor_service = rospy.ServiceProxy(
                self.settings["descriptor_service"], Descriptor)
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
        prediction_text = self.settings.get("class_mapping").get(str(prediction))
        if prediction_text is None:
            self.print_info(f"Unknown class prediction [class mapping is missing]")
            return
        self.decision_publisher.publish(prediction_text)


if __name__ == "__main__":

    rospy.init_node("ecs_classification_classic_node")
    ibec = ImageBasedEnvironmentClassification()
    rospy.spin()
