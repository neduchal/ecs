#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import numpy as np
import torch
import torchvision
#from torch.utils.tensorboard import SummaryWriter
import torch.nn as nn
#import torch.nn.functional as F
import torchvision.models as models
#import torch.optim as optim
import torchvision.transforms as transforms
import pretrainedmodels
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as pImage


class ImageBasedEnvironmentClassification:

    def __init__(self):
        # load parameters
        # ! SUMMARY PARAMETERS IN DOCUMENTATION
        self.settings = None
        self.load_settings()
        rospack = rospkg.RosPack()
        self.models_path = os.path.join(rospack.get_path("ecs"), "models")
        self.network_name = self.settings["network_name"]
        self.network_device = self.settings["network_device"]
        if self.settings.get("number_of_classes") == None:
            self.settings["number_of_classes"] = 2
        self.image_subscriber = rospy.Subscriber(
            self.settings["camera_topic"], Image, callback=self.image_subscriber_callback, queue_size=1)
        self.trigger_subscriber = rospy.Subscriber(
            self.settings["trigger_topic"], Empty, callback=self.trigger_callback, queue_size=1)
        self.decision_publisher = rospy.Publisher(
            self.settings["decision_topic"], String, queue_size=10)
        self.img = None
        self.net = None
        self.cv_bridge = CvBridge()

        self.print_info(f"Network name {self.network_name}")
        self.print_info(f"Network device {self.network_device}")
        self.print_info(f"Number of Clases {self.settings['number_of_classes']}")
        self.print_info(f"Camera topic {self.settings['camera_topic']}")
        self.print_info(f"Trigger topic {self.settings['trigger_topic']}")
        self.print_info(f"Decision_topic {self.settings['decision_topic']}")

        self.load_neural_network()
        if self.network_device == "gpu":
            device = torch.device(
                "cuda:0" if torch.cuda.is_available() else "cpu")
        elif self.network_device == "cpu":
            device = torch.device("cpu")
        else:
            exit(1)
            print("Unkwown device type")
        self.net = self.net.to(device)

        self.loader = transforms.Compose([transforms.ToTensor()])

    def print_info(self, msg):
        rospy.loginfo(f"[{rospy.get_name()}]: {msg}")

    def load_settings(self):
        self.settings = rospy.get_param("ecs_ibec")

    def load_neural_network(self):
        if self.network_name == "vgg16":
            self.net = models.vgg16_bn(pretrained=False)
            num_ftrs = self.net.classifier[6].in_features
            self.net.classifier[6] = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, "VGG16_best.pth")))
        elif self.network_name == "densenet":
            self.net = models.densenet161(pretrained=False)
            num_ftrs = self.net.classifier.in_features
            self.net.classifier = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, 'DenseNet_best.pth')))
        elif self.network_name == "xception":
            self.net = pretrainedmodels.xception()
            num_ftrs = self.net.last_linear.in_features
            self.net.last_linear = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, 'Xception_best.pth')))
        elif self.network_name == "wide_resnet50":
            self.net = models.wide_resnet50_2(pretrained=True)
            num_ftrs = self.net.fc.in_features
            self.net.fc = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, 'ResNetWide_best.pth')))
        elif self.network_name == "resnext":
            self.net = models.resnext50_32x4d(pretrained=True)
            num_ftrs = self.net.fc.in_features
            self.net.fc = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, 'ResNext_best.pth')))
        elif self.network_name == "inceptionresnetv2":
            self.net = models.wide_resnet50_2(pretrained=True)
            num_ftrs = self.net.fc.in_features
            self.net.fc = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, 
         'InceptionResNetv2_best.pth')))
        elif self.network_name == "inceptionv4":
            self.net = pretrainedmodels.inceptionv4()
            self.net.avg_pool = nn.AvgPool2d(kernel_size=2, stride=2)
            num_ftrs = self.net.last_linear.in_features
            self.net.last_linear = nn.Linear(num_ftrs, self.settings["number_of_classes"])
            self.net.load_state_dict(torch.load(os.path.join(self.models_path, 'InceptionV4_best.pth')))
        else:
            self.net = None
            print("unknown neural network architecture")
        self.net.eval()

    def image_subscriber_callback(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="CV_8UC3")

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
            print("Unkwown device type")
            exit(1)
        prediction = predictions.detach().numpy()
        prediction_text =  self.settings.get("class_mapping").get(str(prediction))
        if prediction_text is None:
            self.print_info(f"Unknown class prediction [class mapping is missing]")
            return
        self.decision_publisher.publish(prediction_text)


if __name__ == "__main__":

    rospy.init_node("ecs_classification_classic_node")
    ibec = ImageBasedEnvironmentClassification()
    rospy.spin()
