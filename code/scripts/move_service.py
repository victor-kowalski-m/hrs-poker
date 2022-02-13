#!/usr/bin/env python

"""
This file's purpose is implementing all the movements required for the game.
It does so by setting up a service and responding to requests with the correspondent
movement function. This correspondence is shown in the dictionary "commands".
Each function executes a determined set of moves with necessary pauses inbetween.
The function executes the moves by setting a combination of joints 
("Posture" class instance) or calling the cartesian control API.
"""

from rosgraph.masterapi import Error
import rospy
import numpy as np
from project.srv import Move
import motion
import time
from sensor_msgs.msg import JointState
import atexit
import subprocess
from naoqi_bridge_msgs.msg import HeadTouch
from naoqi import ALProxy

# Motion proxy setup
robotIP = "10.152.246.171"
PORT = 9559
motionProxy = ALProxy("ALMotion", robotIP, PORT)

rhand = 0 # keeps track of right hand state

class Posture:
    """
    Class for representing a certain combination of
    joint angles.
    """

    def __init__(self, dic):
        self.names = dic.keys()
        self.angles = dic.values()


### Definition of Posture instances

legs_joints = Posture(
    {
        "LAnklePitch": 0.24,
        "LAnkleRoll": -0.04,
        "LHipPitch": -1.20,
        "LHipRoll": -0.02,
        "LHipYawPitch": -0.04,
        "LKneePitch": 0.92,

        "RAnklePitch": 0.20,
        "RAnkleRoll": 0.24,
        "RHipPitch": -1.21,
        "RHipRoll": 0.02,
        "RHipYawPitch": -0.04,
        "RKneePitch": 0.87
    }
)

holding_joints = Posture(
    {
        "HeadPitch": 0.1,
        "HeadYaw": -0.60,

        "LElbowRoll": -1.38,
        "LElbowYaw": -1.73,
        "LHand": 0,
        "LShoulderPitch": 1.50,
        "LShoulderRoll": 0.29,
        "LWristYaw": -0.35,

        "RElbowRoll": 1.31,
        "RElbowYaw": 1.61,
        "RHand": 0.08,
        "RShoulderPitch": 0.64,
        "RShoulderRoll": -0.00,
        "RWristYaw": 1.27,
    }
)

fold_joints = Posture(
    {
        "HeadPitch": -0.05,
        "HeadYaw": -0.23,

        "RElbowRoll": 0.77,
        "RElbowYaw": -0.10,
        "RHand": 0.01,
        "RShoulderPitch": -0.3,
        "RShoulderRoll": -0.33,
        "RWristYaw": 0,


    }
)

knock_joints = [
    Posture(
        {
            "HeadPitch": -0.03,
            "HeadYaw": -0.04,

            "LElbowRoll": -1.53,
            "LElbowYaw": -0.64,
            "LHand": 0,
            "LShoulderPitch": 1.02,
            "LShoulderRoll": 0.37,
            "LWristYaw": 0.82,

        }
    ),
    Posture(   
        {
            "HeadPitch": -0.03,
            "HeadYaw": -0.04,

            "LElbowRoll": -1.45,
            "LElbowYaw": -0.5,
            "LHand": 0,
            "LShoulderPitch": 1.18,
            "LShoulderRoll": 0.32,
            "LWristYaw": 0.85,
        }
    )
]


scan_table_joints = Posture(
  {
        "HeadPitch": 0.00,
        "HeadYaw": 0.2,

        "LElbowRoll": -1.38,
        "LElbowYaw": -1.73,
        "LHand": 0,
        "LShoulderPitch": 1.50,
        "LShoulderRoll": 0.29,
        "LWristYaw": -0.35,

        "RElbowRoll": 1.33,
        "RElbowYaw": 1.67,
        "RHand": 0,
        "RShoulderPitch": 1.28,
        "RShoulderRoll": -0.13,
        "RWristYaw": 0.16,
    }
)

show_joints = Posture(
    {

        "RElbowRoll": 0.04,
        "RElbowYaw": 1.29,
        "RHand": 0,
        "RShoulderPitch": 0.23,
        "RShoulderRoll": 0.04,
        "RWristYaw": 1.82,
    }
)

rest_joints = Posture(
    {
        "HeadPitch": -0.00,
        "HeadYaw": -0.00,

        "LElbowRoll": -1.38,
        "LElbowYaw": -1.73,
        "LHand": 0,
        "LShoulderPitch": 1.50,
        "LShoulderRoll": 0.29,
        "LWristYaw": -0.35,

        "RElbowRoll": 1.33,
        "RElbowYaw": 1.67,
        "RHand": 0,
        "RShoulderPitch": 1.28,
        "RShoulderRoll": -0.13,
        "RWristYaw": 0.16,
    }
)

rest_left_joints = Posture(
    {
        "HeadPitch": -0.00,
        "HeadYaw": -0.00,

        "LElbowRoll": -1.38,
        "LElbowYaw": -1.73,
        "LHand": 0,
        "LShoulderPitch": 1.50,
        "LShoulderRoll": 0.29,
        "LWristYaw": -0.35,

    }
)


### Helper function for raise movements

def get_aruco():
    """
    Gets aruco position from parameter server.
    Outputs:
        aruco: 6D array representing the marker position
    """

    try:
        aruco = np.array(rospy.get_param("chip_aruco_pos"))
    except:
        print("Don't know where aruco is.")
        deny_move()
        raise Error
    return aruco


def go_to_chip(depth, stack, height, aruco, sleep):
    """
    Moves left hand to a certain block of 5 chips.
    Inputs:
        depth: how deep to extend the arm (a value to be added to the marker's x coordinate)
        stack: which stack to reach (0-2)
        height: which height to reach (0-3)
        aruco: 6D array representing the marker position
        sleep: how long to sleep after executing
    """

    chipPos = np.array([depth, stack*0.07, 0.05+height*0.02, 0, 0, 0])
    motionProxy.setPositions("LArm", motion.FRAME_TORSO, (aruco+chipPos).tolist(), 0.8, 7)
    time.sleep(sleep)


### Functions for executing movements

def initial_move():
    """
    Executes initial movement.
    """

    motionProxy.setAngles(legs_joints.names, legs_joints.angles, 0.05)
    motionProxy.setAngles(holding_joints.names, holding_joints.angles, 0.3)
    time.sleep(1)


def check_move():
    """
    Executes check movement.
    """

    motionProxy.setAngles(knock_joints[0].names, knock_joints[0].angles, 0.3)
    time.sleep(0.5)
    for i in [1, 0]*2:
        motionProxy.setAngles(knock_joints[i].names, knock_joints[i].angles, 1)
        time.sleep(0.2)
    rest_left_move()


def fold_move():
    """
    Executes fold movement.
    """

    motionProxy.setAngles(fold_joints.names, fold_joints.angles, 0.3)
    time.sleep(1)
    open_close_move("o")
    rest_move()


def show_move():
    """
    Executes fold movement.
    """

    motionProxy.setAngles(show_joints.names, show_joints.angles, 0.3)
    time.sleep(1)
    open_close_move("o")
    rest_move()


def raise_move(stack, height):
    """
    Executes raise movement.
    Inputs:
        stack: which stack to reach (0-2) (0 is the one right above the aruco marker)
        height: which height to reach (0-3) 
    """

    try:
        aruco = get_aruco()
    except:
        return
    
    depth = [-0.1, -0.03]
    height_add = [1, 0]
    for i in [0, 1, 0]:
        go_to_chip(depth[i], stack, height_add[i]+height, aruco, 1)

    motionProxy.setAngles(knock_joints[0].names, knock_joints[0].angles, 0.3)
    time.sleep(1)
    rest_left_move()


def open_close_move(which="switch"):
    """
    Open, closes or switches right hand.
    Inputs:
        which: "o", "c" or "switch"
    """

    angle = {
        "switch": int(not rhand),
        "o": 1,
        "c": 0
    }
    motionProxy.setAngles("RHand", [angle[which]], 0.5)
    time.sleep(0.5)


def all_in_move(furthest_stack):
    """
    Executes all in movement.
    Inputs:
        furthest_stack: is the one from which the movement starts
    """

    try:
        aruco = get_aruco()
    except:
        return

    if furthest_stack == 0:
        raise_move(0, 0)
    else:
        before = -0.1
        above = -0.03
        go_to_chip(before, furthest_stack+1/furthest_stack, -1, aruco, 1)
        go_to_chip(above, furthest_stack+1/furthest_stack, -1, aruco, 1)
        motionProxy.setAngles(["LHand", "LWristYaw"], [1, -0.4], 0.5)
        time.sleep(0.5)
        go_to_chip(above, -0.5, 0, aruco, 1)
        go_to_chip(before, furthest_stack, 0, aruco, 1)
        rest_left_move()


def scan_table_move():
    """
    Executes scan table movement.
    """

    motionProxy.setAngles(scan_table_joints.names, scan_table_joints.angles, 0.2)
    time.sleep(1)

def nod_move():
    """
    Executes nod movement.
    """

    motionProxy.setAngles(["HeadPitch"], [0.3], 0.3)
    time.sleep(0.3)
    motionProxy.setAngles(["HeadPitch"], [0], 0.3)
    time.sleep(0.3)

def deny_move():
    """
    Executes deny movement.
    """

    motionProxy.setAngles(["HeadYaw"], [0.2], 0.5)
    time.sleep(0.2)
    motionProxy.setAngles(["HeadYaw"], [-0.2], 0.5)
    time.sleep(0.3)
    motionProxy.setAngles(["HeadYaw"], [0.2], 0.5)
    time.sleep(0.3)
    motionProxy.setAngles(["HeadYaw"], [0], 0.2)
    time.sleep(0.5)

def rest_move():
    """
    Executes rest movement.
    """

    motionProxy.setAngles(rest_joints.names, rest_joints.angles, 0.3)
    time.sleep(1)

def rest_left_move():
    """
    Rests left hand only.
    """

    motionProxy.setAngles(rest_left_joints.names, rest_left_joints.angles, 0.3)
    time.sleep(1)


## Service handling definition

# Possible commands
commands = {
    "ai": all_in_move,
    "c": check_move,
    "d": deny_move,
    "f": fold_move,
    "i": initial_move,
    "n": nod_move,
    "oc": open_close_move,
    "ra": raise_move,
    "re": rest_move,
    "sh": show_move,
    "st": scan_table_move
}


def service_handle(move):
    """
    Handle for the received requests.
    Inputs:
        move: srv of type Move representing the desired movement
    Outputs:
        "ok"
    """
    
    words = move.name.split(" ")
    if len(words)==2:
        commands[words[0]](int(words[1]))
    if len(words)==3:
        commands[words[0]](int(words[1]), int(words[2]))
    else:
        commands[words[0]]()
    return "ok"


def joint_state_cb(data):
    """
    Keeps track of right hand state.
    Inputs:
        data: msg containing the joint_states
    """

    global rhand
    
    angle = data.position[data.name.index("RHand")]
    rhand = int(angle > 0.5)


def tactile_cb(data):
    """
    Switches hand state on touch of middle button.
    Inputs:
        data: msg containing the pressed button state
    """

    if data.state:
        if data.button == 2:
            motionProxy.setAngles("RHand", [int(not rhand)], 0.5)
            time.sleep(0.5)


def exit_disable_stiffness():
    """
    Function executed on exit of script, goes to rest position 
    and disables stiffness.
    """

    open_close_move("o")
    rest_move()
    subprocess.check_call("rosservice call /body_stiffness/disable",shell=True)


def main():

    rospy.init_node('move_service_node',anonymous=True)
    
    atexit.register(exit_disable_stiffness)
    subprocess.check_call("rosservice call /body_stiffness/enable",shell=True)

    rospy.Subscriber("/joint_states", JointState, joint_state_cb)
    rospy.Subscriber("/tactile_touch", HeadTouch, tactile_cb)
    rospy.Service('move_service', Move, service_handle)

    print("Ready to move.")
    rospy.spin()


if __name__=='__main__':
    main()
