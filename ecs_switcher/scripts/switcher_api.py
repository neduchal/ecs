#!/usr/bin/env python
import rospy
import roslaunch
import rospkg
import os.path
from std_msgs.msg import String


def start_node(package, executable):
    node = roslaunch.core.Node(package, executable)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    return process


def start_launch_file(launch_file):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
    launch.start()
    return launch


def stop_node(process):
    process.stop()


def stop_launch(launch):
    launch.shutdown()


def get_path_to_pkg(pkg_name):
    rospack = rospkg.RosPack()
    return rospack.get_path(pkg_name)


def get_launch_from_pkg(pkg_name, launch_filename):
    pkg = get_path_to_pkg(pkg_name)
    return os.path.join(pkg, "launch", launch_filename)


def find_and_launch(pkg_name, launch_filename):
    launch_file = get_launch_from_pkg(pkg_name, launch_filename)
    launch_file_arr = []
    launch_file_arr.append(launch_file)
    return start_launch_file(launch_file_arr)
