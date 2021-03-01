#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np 
from joblib import load
from sklearn import svm
from std_msgs.msg import Empty

class ImageBasedEnvironmentClassification:

    def __init__(self):
        # load parameters
        self.classifier_file = rospy.get_param("/ecs/classifier_classic", default="default.joblib")

        self.classifier = load(self.classifier_file)

        self.trigger_subscriber = rospy.Subscriber("/ecs/trigger", Empty, callback=self.trigger_callback, queue_size=1)
        pass

    def trigger_callback(self, msg):
        pass

    def process(self, im):

        # call service /ecs/descriptor

        predict = clf.predict(desc_vector.reshape(1, -1))[0]

        # publish prediction


if __name__ == "__main__":

    rospy.init_node("ecs_classification_classic_node")

    ibec = ImageBasedEnvironmentClassification()

    rospy.spin()
