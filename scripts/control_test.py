#!/usr/bin/env python
import rospy
import numpy as np
import math
from scipy import interpolate
import matplotlib.pyplot as plt
from lowlevelmotion import LowLevelMotion

if __name__ == '__main__':
    try:
        # define some way pointsi:
        # [xp, yp, zp, wq, xq, yq, zq]
        wayPoint0 = [0, 0.6, 0.06, 1, 0, 0, 0]
        wayPoint1 = [0, 0.6, 0.05, 1, 0, 0, 0]
        wayPoint2 = [0, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint3 = [-0.2, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint4 = [-0.2, 0.6, 0.05, 1, 0, 0, 0]
        wayPoints = [wayPoint1, wayPoint2, wayPoint3, wayPoint4]
        wayPoint_pause = [0, 0.6, 0.4, 1, 0, 0, 0]

        # define some pattern parameters:
        lift_height = 0.1 # height after one pressure completed
        move_speed = 0.2 # ratio to the max speed
        press_duration = 2 # seconds
        press_force = 20 # the vibrating pressing force, N
        vibration_freq = 100; # vibration frequence
        vibration_repetition = 20; # vibration repetition of one vibration

        # initialize a LowLevelMotion object
        arm = LowLevelMotion()
        
        # move the arm to zero position first
        arm.moveToZero()
        arm.moveToPoint(wayPoint_pause)
        # configure the force press issue
        #arm.prePress(wayPoint0)

        # start the massage pattern:
        #arm.normalPress(wayPoints, lift_height, move_speed, press_duration)

        # start the repeated normal press
        #arm.repeatedPress(wayPoints, 0.04, move_speed, 5, 1)
        # configure the force press issue
        #arm.prePress(wayPoint1)

        arm.prePress(wayPoint1)
        # start vibration massage pattern
        arm.armVibrate(wayPoints, lift_height, move_speed, vibration_freq, vibration_repetition)

        # arm.prePress(wayPoint1)
        # start circular massage pattern
        # arm.circularMotion(wayPoints, lift_height, move_speed, True)

    except rospy.ROSInterruptException:
	pass
