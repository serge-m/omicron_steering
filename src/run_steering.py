#!/usr/bin/python3

"""
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import os
import json

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32

# GLOBAL VARIABLES
DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.join(DIR, "..", "../steering.json")

CONFIG = dict()


def set_variables():
    if not os.path.exists(CONFIG_DIR):
        raise ("CONFIG does not exist! Please calibrate your car first!")

    with open(CONFIG_DIR) as f:
        d = json.load(f)

        CONFIG["STEERING_LEFT_PWM"] = d["STEERING_LEFT_PWM"]
        CONFIG["STEERING_RIGHT_PWM"] = d["STEERING_RIGHT_PWM"]

def map_range(x, x_min, x_max, y_min, y_max):
    """
    Linear mapping between two ranges of values
    """
    x = float(x)
    x_range = x_max - x_min
    y_range = y_max - y_min
    xy_ratio = x_range / y_range

    y = ((x - x_min) / xy_ratio + y_min)

    return int(y)


class ROSPackage_Steering:
    LEFT_ANGLE = 0.51
    RIGHT_ANGLE = -0.51

    def __init__(self):
        self.steering_publisher = rospy.Publisher('channel_00', Int32, queue_size=10)

    def callback(self, data):
        steering = data.drive.steering_angle

        pulse = map_range(steering, self.LEFT_ANGLE, self.RIGHT_ANGLE, CONFIG["STEERING_LEFT_PWM"], CONFIG["STEERING_RIGHT_PWM"])
        self.steering_publisher.publish(pulse)

    def run(self):
        rospy.init_node('steering', anonymous=False)
        rospy.Subscriber('ackermann_cmd_mux/output', AckermannDriveStamped, self.callback)
        rospy.spin()


if __name__ == '__main__':
    set_variables()

    package = ROSPackage_Steering()
    package.run()
