#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


class DiffRatio:

    def __init__(self, sensitivity_threshold=1.4, min_length=40):
        self.sensitivity_threshold_ = sensitivity_threshold
        self.short_term_window_ = []
        self.long_term_window_ = []
        self.min_length_ = min_length

    def addValue(self, value):
        self.short_term_window_.append(value)
        if len(self.short_term_window_) > self.min_length_ :
            self.short_term_window_.pop(0)
        self.long_term_window_.append(value)

    def isReady(self):
        if len(self.short_term_window_) == self.min_length_ :
            return True
        return False

    def isChangePoint(self):
        if not self.isReady():
            return False
        short_mean = np.mean(self.short_term_window_)
        long_mean = np.mean(self.long_term_window_)
        ratio = short_mean/float(long_mean)
        trigger = np.max([ratio, 1/ratio])
        if trigger > self.sensitivity_threshold_:
            if len(self.long_term_window_) > self.min_length_:
                self.long_term_window_ = self.long_term_window_[-self.min_length_:]
            return True
        return False


class VarRatio:

    def __init__(self, var_stable, sensitivity_threshold=1.2, window_length=40):
        self.sensitivity_threshold_ = sensitivity_threshold
        self.var_stable_ = var_stable
        self.window_ = []
        self.window_length_ = window_length

    def addValue(self, value):
        self.window_.append(value)
        if len(self.window_) > self.window_length_:
            self.window_.pop(0)

    def isReady(self):
        if len(self.window_) == self.window_length_:
            return True
        return False

    def isChangePoint(self):
        if not self.isReady():
            return False        
        trigger = np.var(self.window_)
        if trigger > (self.sensitivity_threshold_ * self.var_stable_):
            return True
        return False
