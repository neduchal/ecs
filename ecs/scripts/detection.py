#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import copy
from scipy import signal
from scipy.interpolate import splrep, splev
from cv_bridge import CvBridge, CvBridgeError
import cv2

def cusum_variance(x,mean=0,K=0):
    """Tabular CUSUM per Montgomery,D. 1996 "Introduction to Statistical Process Control" p318 
    x    : series to analyze
    mean : expected process mean
    K    : reference value, allowance, slack value-- suggest K=1/2 of the shift to be detected.

    Returns:
    value  variance of positive and negative CUSUM
    """
    Cp=(x*0).copy()
    Cm=Cp.copy()
    for ii in np.arange(len(x)):
        if ii == 0:
            Cp[ii]=x[ii]
            Cm[ii]=x[ii]
        else:
            Cp[ii]=np.max([0,x[ii]-(mean+K)+Cp[ii-1]])
            Cm[ii]=np.max([0,(mean-K)-x[ii]+Cm[ii-1]])
    return np.max(Cp - Cm)            

def dirat(x, n_short, threshold=1.5, c=0.03):
    dx1 = np.mean(x[1:] - x[:-1])
    dx2 = np.mean(x[-(n_short-1):] - x[-n_short:-1])
    if (dx2 == 0): 
        return 1.0   
    ratio = np.abs((dx1+c)/np.float(dx2+c))
    if (ratio == 0): 
        return 1.0    
    return np.max([ratio, 1/ratio]) > threshold

def var_sig(x, stable_variance, alpha=1.0):
    return np.var(x) > alpha * stable_variance
