#!/usr/bin/env python

import argparse
import rospy
import externaldevice
import rospy
import numpy as np
import math
from scipy import interpolate
import matplotlib.pyplot as plt
from lowlevelmotion import LowLevelMotion
from geometry_msgs.msg import Pose, Twist, Wrench
from intera_core_msgs.msg import EndpointState
from kitt.msg import Hotword
import signal

class massager(object):
    def __init__(self):
        # all massage controlling parameters
	self._count = 0
        '''
        self._wayPoint0 = [0, 0.6, 0.06, 1, 0, 0, 0]
        self._wayPoint1 = [0, 0.6, 0.05, 1, 0, 0, 0]
        self._wayPoint2 = [0, 0.8, 0.05, 1, 0, 0, 0]
        self._wayPoint3 = [-0.2, 0.8, 0.05, 1, 0, 0, 0]
        self._wayPoint4 = [-0.2, 0.6, 0.05, 1, 0, 0, 0]
        '''
        self._wayPoint0 = [0, 0.65, 0.35, 1, 0, 0, 0]

        self._wayPoint1 = [0.38, 0.73, 0.25, 1, 0, 0, 0]
        self._wayPoint2 = [0.32, 0.62, 0.25, 1, 0, 0, 0]
        self._wayPoint3 = [0.32, 0.73, 0.25, 1, 0, 0, 0]
        self._wayPoint4 = [0.04, 0.77, 0.25, 1, 0, 0, 0]
        self._wayPoint5 = [0.1, 0.64, 0.25, 1, 0, 0, 0]
        self._wayPoints = [self._wayPoint1, self._wayPoint2, self._wayPoint3, self._wayPoint4, self._wayPoint5]
        self._wayPoint_pause = [0, 0.6, 0.5, 1, 0, 0, 0]

        # define some pattern parameters:
        self._lift_height = 0.1 # height after one pressure completed
        self._move_speed = 0.2 # ratio to the max speed
        self._press_duration = 2 # seconds
        self._press_force = 20 # the vibrating pressing force, N
        self._vibration_freq = 100; # vibration frequence
        self._vibration_repetition = 20; # vibration repetition of one vibration
        
        self._arm = LowLevelMotion()
        

    def keyboardCallback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.pose.position.x, 
        #data.pose.position.y, data.pose.position.z)
        c = externaldevice.getch()
        if c:
            if c in ['\x1b', '\x03']:
                rospy.signal_shutdown("finished")
            elif c in ['s', 'S']:
                rospy.loginfo('start!')
            elif c in ['e', 'E']:
                rospy.loginfo('stop!')
            else:
                print(self._count)
                self._count += 1
        # rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.pose.position.x,
        # data.pose.position.y, data.pose.position.z)
        # print("Save EndEffector Position and Orientation!")
    
    def voiceControlCallback(self, data):
        if data.control_msg == 'start':
            # print('start command')
            self._arm.stop = False
            self._arm.moveToZero()
            self._arm.prePress(self._wayPoint0)
            self._arm.armVibrate(self._wayPoints, self._lift_height, self._move_speed, self._vibration_freq, self._vibration_repetition, self._wayPoint_pause)
            self._arm.moveToPoint(self._wayPoint_pause)
        elif data.control_msg == 'controltest':
            self._arm.stop = False
            self._arm.normalPress([self._wayPoint0], self._lift_height, self._move_speed, 4, 20)
        else:
            print('no command')
    
    def voiceInterruptCallback(self, data):
        if data.control_msg == 'harder':
            if self._arm.vibration_force < 50: 
                self._arm.vibration_force += 10
            print('harder')
        elif data.control_msg == 'lighter':
            if self._arm.vibration_force > 30:
                self._arm.vibration_force -= 10
            print('lighter')
        elif data.control_msg == 'pause':
            self._arm.stop = True
            print('pause')
        else:
            print('no command')
    
    def stop(self):
        # do something
        print('stop')

    def keyboardListener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('keyboardlistener', anonymous=True)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.keyboardCallback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def voiceListener(self):
        rospy.init_node('voicelistener', anonymous=True)

        rospy.Subscriber('/hotword', Hotword, self.voiceControlCallback)
        rospy.Subscriber('/hotwordInterrupt', Hotword, self.voiceInterruptCallback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    '''
    epilog = """
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description='recorder format',
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    '''

    #helper = massager(args.filename)
    helper = massager()
    # helper.keyboardlistener()
    helper.voiceListener()
    rospy.on_shutdown(helper.stop)
    #externaldevice.getch()
    print 'Done'

