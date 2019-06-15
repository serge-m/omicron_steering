#!/usr/bin/env python

import os
import json

import rospy
from std_msgs.msg import Int32

from dynamic_reconfigure.server import Server
from steering.cfg import SteeringConfig

# GLOBAL VARIABLES
DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.join(DIR, "..", "../steering.json")

CONFIG = dict()

PUBLISHER = rospy.Publisher('channel_00', Int32, queue_size=10)


def set_variables():
    global CONFIG
    if not os.path.exists(CONFIG_DIR):
        CONFIG["STEERING_LEFT_PWM"] = 0
        CONFIG["STEERING_RIGHT_PWM"] = 0

    else:
        with open(CONFIG_DIR) as f:
            CONFIG = json.load(f)


def dump_data():
    with open(CONFIG_DIR, 'w') as outfile:
        json.dump(CONFIG, outfile)


def callback(received_config, level):
    rospy.loginfo(
        """Current CONFIG: {STEERING_LEFT_PWM} {STEERING_RIGHT_PWM}""".format(**CONFIG))
    rospy.loginfo("""Reconfigure Request: {STEERING_LEFT_PWM} {STEERING_RIGHT_PWM}""".format(**received_config))

    # only one at the time will change and we need to publish it
    if CONFIG["STEERING_LEFT_PWM"] != received_config["STEERING_LEFT_PWM"]:
        CONFIG["STEERING_LEFT_PWM"] = received_config["STEERING_LEFT_PWM"]
        PUBLISHER.publish(received_config["STEERING_LEFT_PWM"])

    elif CONFIG["STEERING_RIGHT_PWM"] != received_config["STEERING_RIGHT_PWM"]:
        CONFIG["STEERING_RIGHT_PWM"] = received_config["STEERING_RIGHT_PWM"]
        PUBLISHER.publish(received_config["STEERING_RIGHT_PWM"])

    dump_data()

    return received_config


def main():
    srv = Server(SteeringConfig, callback)
    rospy.spin()


if __name__ == '__main__':
    set_variables()
    rospy.init_node('config_steering', anonymous=False)

    try:
        main()
    except rospy.ROSInterruptException:
        pass
