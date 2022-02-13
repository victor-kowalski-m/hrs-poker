#!/usr/bin/env python

"""
This file's purpose is detecting the cards on the robot's hand
and publishing them to the correspondent topic (/hand_cards).
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import detection
from project.msg import CardList

hand_pub = None # Card publisher

params = { # Card detection parameters
    "CARD_MAX_AREA": 50000,
    "CARD_MIN_AREA": 5000,
    "THRESH_LEVEL": 150
}

def detect_hand(data):
    """
    Detect and publishes a list of cards captured by NAO's top camera.
    Inputs:
        data: matrix of image captured by NAO's camera
    """

    bridge_instance = CvBridge()
    try:
        image = bridge_instance.imgmsg_to_cv2(data,"bgr8")  
    except CvBridgeError as e:
        rospy.logerr(e)
    hand_card_list = detection.card_detector(image, "Hand", params)
    hand_pub.publish(hand_card_list)

    cv2.waitKey(1)


def main():
    global hand_pub

    rospy.init_node('detect_hand',anonymous=True)
    detection.load_train()
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, detect_hand)
    hand_pub = rospy.Publisher('hand_cards', CardList, queue_size=10)
    rospy.spin()
    

if __name__=='__main__':
    main()