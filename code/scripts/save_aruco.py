#!/usr/bin/env python

"""
This file's purpose is setting up a service for transforming a requested
aruco marker position from bottom camera optical frame to torso frame
and saving it on the parameter server.
"""

import rospy
import motion
import numpy as np
from naoqi import ALProxy
from project.srv import SavePosition
import sys

robotIP = "10.152.246.171"
PORT = 9559
motionProxy = ALProxy("ALMotion", robotIP, PORT)

found = 0

def service_handle(req):
    """
    Transforms aruco position from bottom camera optical frame to torso frame
    and saves it on parameter server.
    Input 
        req: srv of type SavePosition, defined on srv folder
    """

    global found
    position_optical = np.array([req.desired.linear.x, req.desired.linear.y, req.desired.linear.z]).reshape(3,1) # Marker position in bottom cam optical frame

    # Bottom cam optical frame to bottom camera frame, by visual inspection on RViz
    optical2bottom = np.array([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0]
    ])
    position_bottom = optical2bottom.dot(position_optical)

    # Bottom camera frame to torso frame
    bottom2torso = motionProxy.getTransform('CameraBottom', motion.FRAME_TORSO, True)
    bottom2torso = np.reshape(bottom2torso, (4, 4))
    position_bottom_hom = np.append(position_bottom, np.array([[1]]), axis=0)
    position_torso_hom = bottom2torso.dot(position_bottom_hom)

    # Saves position to parameter server
    aruco_pos = np.append(np.reshape(position_torso_hom[0:3], 3), np.zeros(3))
    rospy.set_param("chip_aruco_pos", aruco_pos.tolist())
    found = 1
    return found


if __name__ == '__main__':
    rospy.init_node('save_aruco_service')
    rospy.Service('save_aruco_service', SavePosition, service_handle)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if found:
            rospy.loginfo("Found Aruco. Exiting.")
            sys.exit(0)
        rate.sleep()
			
		
