#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import detection as d
import numpy as np


class sensor:

    def __init__(self, method, parameters):
        self.data = []
        self.method = method
        self.parameters = parameters
        self.stable_params = {}
        pass

    def add(self, value):
        self.data.append(value)
        if len(self.data) > 3 * self.parameters["number_of_samples"]:
            del self.data[0]

    def ready(self):
        if len(self.data) == 3 * self.parameters["number_of_samples"]:
            return True
        return False

    def calibrate(self, duration):
        if len(self.data) == 3 * self.parameters["number_of_samples"]:
            self.stable_params["mean"] = np.mean(self.data)
            self.stable_params["variance"] = np.var(self.data)

    def compute(self):
        if not self.ready:
            rospy.loginfo("Sensor %s processing is not ready", self.parameters["name"])
            return 0
        if self.method == "cusum":
            value = d.cusum_variance(self.data[-self.parameters["number_of_samples"]:],
                                     self.parameters["threshold"], np.mean(self.data), self.parameters["K"])
            pass
        elif self.method == "dirat":
            value = d.dirat(self.data[-self.parameters["number_of_samples"]:],
                            self.parameters["n_short"], self.parameters["threshold"], self.parameters["c"])
            return value

        elif self.method == "varsig":
            value = d.var_sig(self.data[self.parameters["number_of_samples"]:],
                              self.stable_params["variance"], self.parameters["alpha"])
            return value
        else:
            rospy.logwarn("Unknown Method")
        pass
