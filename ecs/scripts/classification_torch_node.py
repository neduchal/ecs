#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np 
from joblib import load
import torch
import torchvision
from torch.utils.tensorboard import SummaryWriter
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models
import torch.optim as optim
import torchvision.transforms as transforms
import pretrainedmodels
from std_msgs.msg import Empty, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as pImage


class ImageBasedEnvironmentClassification:

    def __init__(self):
        # load parameters
        self.network_name = rospy.get_param("/ecs/neural_network_name", default="vgg16")
        self.network_device = rospy.get_param("/ecs/neural_network_device", default="cpu")
        self.num_classes = 2
        self.image_subscriber = rospy.Subscriber("/ecs/image", Image, callback=self.image_subscriber_callback, queue_size=1)
        self.trigger_subscriber = rospy.Subscriber("/ecs/trigger", Empty, callback=self.trigger_callback, queue_size=1)
        self.decision_publisher = rospy.Publisher("/ecs/decision", Int8, queue_size=10)
        self.img = None
        self.net = None
        self.cv_bridge = CvBridge()

        self.load_neural_network()
        if self.network_device == "gpu":
            device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        elif self.network_device == "cpu": 
            device = torch.device("cpu")
        else:
            exit(1)
            print("Unkwown device type")
        self.net = self.net.to(device)

        self.loader = transforms.Compose([ transforms.ToTensor()])

        

    def load_neural_network(self):
        if self.network_name == "vgg16":
            self.net = models.vgg16_bn(pretrained=False)
            num_ftrs = self.net.classifier[6].in_features
            self.net.classifier[6] = nn.Linear(num_ftrs, self.num_classes)
            self.net.load_state_dict(torch.load('models/VGG16_best.pth'))
        elif self.network_name == "densenet":
            self.net = models.densenet161(pretrained=False)
            num_ftrs = self.net.classifier.in_features
            self.net.classifier = nn.Linear(num_ftrs, self.num_classes)
            self.net.load_state_dict(torch.load('models/DenseNet_best.pth'))
        elif self.network_name == "xception":
            self.net = pretrainedmodels.xception()
            num_ftrs = self.net.last_linear.in_features
            self.net.last_linear = nn.Linear(num_ftrs, self.num_classes)
            self.net.load_state_dict(torch.load('models/Xception_best.pth'))
        elif self.network_name == "wide_resnet50":
            self.net = models.wide_resnet50_2(pretrained=True)
            num_ftrs = self.net.fc.in_features
            self.net.fc = nn.Linear(num_ftrs, self.num_classes)          
            self.net.load_state_dict(torch.load('models/ResNetWide_best.pth'))         
        elif self.network_name == "resnext":
            self.net = models.resnext50_32x4d(pretrained=True)
            num_ftrs = self.net.fc.in_features
            self.net.fc = nn.Linear(num_ftrs, self.num_classes)
            self.net.load_state_dict(torch.load('models/ResNext_best.pth'))
        elif self.network_name == "inceptionresnetv2":
            self.net = models.wide_resnet50_2(pretrained=True)
            num_ftrs = self.net.fc.in_features
            self.net.fc = nn.Linear(num_ftrs, self.num_classes) 
            self.net.load_state_dict(torch.load('models/InceptionResNetv2_best.pth'))                
        elif self.network_name == "inceptionv4":
            self.net = pretrainedmodels.inceptionv4()
            self.net.avg_pool = nn.AvgPool2d(kernel_size=2, stride =2)
            num_ftrs = self.net.last_linear.in_features
            self.net.last_linear = nn.Linear(num_ftrs, self.num_classes)
            self.net.load_state_dict(torch.load('models/InceptionV4_best.pth'))
        else:
            self.net = None
            print("unknown neural network architecture")
        self.net.eval()        

    def image_subscriber_callback(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="CV_8UC3")

    def trigger_callback(self, msg):
        self.process()

    def process(self):
        image = pImage.fromarray(np.uint8(self.img))
        image = self.loader(image).float()
        image = image.unsqueeze(0) 
        if self.network_device == "gpu":
            predictions = self.net(image.cuda()).cpu().argmax()
        elif self.network_device == "cpu":
            predictions = self.net(image).cpu().argmax()
        else:
            exit(1)
            print("Unkwown device type")   
            prediction = predictions.detach().numpy()             
        self.decision_publisher.publish(prediction)

if __name__ == "__main__":

    rospy.init_node("ecs_classification_classic_node")
    ibec = ImageBasedEnvironmentClassification()
    rospy.spin()
