#!/usr/bin/env python

"""
This file's purpose is detecting the cards on the table
and publishing them to the correspondent topic (/table_cards).
"""

import rospy
import cv2
import detection
from project.msg import CardList
import subprocess

table_pub = None # Card publisher

params = { # Card detection parameters
    "CARD_MAX_AREA": 50000,
    "CARD_MIN_AREA": 5000,
    "THRESH_LEVEL": 60
}

# Sets up webcam
webcam = cv2.VideoCapture(0)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_auto=1",shell=True) # 1 manual 3 auto
subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_absolute=100",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c focus_auto=0",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c focus_absolute=20",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c zoom_absolute=130",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c gain=0",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c sharpness=255",shell=True)

def detect_table():
    """
    Detect and publishes a list of cards captured by the webcam.
    """

    ret, image = webcam.read()
    table_card_list = detection.card_detector(image, "Table", params)
    table_pub.publish(table_card_list)

    cv2.waitKey(1)

def main():
    global table_pub

    rospy.init_node('detect_table',anonymous=True)
    detection.load_train()
    table_pub = rospy.Publisher('table_cards', CardList, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        detect_table()
        rate.sleep()


if __name__=='__main__':
    main()