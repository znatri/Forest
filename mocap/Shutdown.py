# from rtpmidi import RtpMidi
# from pymidi import server
import os
import sys
import time
import csv
import pdb
from queue import Queue
from threading import Thread
import time
import numpy as np



def setup():
    i = 0
    for a in arms:
        print("robot",i)
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 1.57, 0.0, 0, 0.0], wait=False, speed=0.2, acceleration=0.1,
                          is_radian=True)
        i += 1
if __name__ == "__main__":
    ROBOT = "xArms"
    PORT = 5004

    sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
    from xarm.wrapper import XArmAPI

    # arm1 = XArmAPI('192.168.1.211')
    arm1 = XArmAPI('192.168.1.203')
    arm2 = XArmAPI('192.168.1.242')
    arm3 = XArmAPI('192.168.1.236')
    arm4 = XArmAPI('192.168.1.244')
    arm5 = XArmAPI('192.168.1.234')
    arm6 = XArmAPI('192.168.1.215')
    arm7 = XArmAPI('192.168.1.208')
    arm8 = XArmAPI('192.168.1.226')
    arm9 = XArmAPI('192.168.1.211')
    # arm10 = XArmAPI('192.168.1.204')


    # arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm10]
    arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9]
    # arms = [arm1, arm2]
    totalArms = len(arms)
    for a in arms:
        print("robot")
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0], wait=False, speed=0.2, acceleration=0.1,
                          is_radian=True)
    input("ready to shutdown?")

    for a in arms:
        a.motion_enable(enable=False)

