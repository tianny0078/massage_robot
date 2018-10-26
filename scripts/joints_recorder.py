#!/usr/bin/env python
import rospy
import math
import time

from std_msgs.msg import (
    String,
    Float64,
    Header
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import (
    JointCommand,
    EndpointState
)

class recorder(object):

    def __init__(self):
        self.listOfJoints = []
        self.counter = 0
        self.recordedData = open("listOfJoints.txt", 'w')

    def callback(self, data):
        # update the counter
        self.counter += 1
        
        if self.counter%50==0:
            rospy.logerr("recorded")
            self.listOfJoints.append(data.position[1:8])
            self.recordedData.write(str(data.position[1:8]))
            self.recordedData.write("\n")
            print(self.listOfJoints)
        rospy.loginfo("skipped")

        if self.counter >= 1000:
            self.recordedData.close()
        #self.recordedData.write(str(data.position))

    def listener(self):
        rospy.init_node('jointStateListener', anonymous=True)
        #self.recordedData = open("listOfJoints.txt", 'w')
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.callback)
        rospy.loginfo('listen')
        rospy.spin()

if __name__ == '__main__':
    try:
        new_recorder = recorder()
        new_recorder.listener()
    except:
        pass

