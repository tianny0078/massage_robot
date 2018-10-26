#!/usr/bin/env python
import lowlevelmotion
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dmp.srv import *
from dmp.msg import *
import math

from std_msgs.msg import (
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
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    # initialize a dmp trajectory message
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        # initialize a dmp point
        pt = DMPPoint();
        # assign the points in the trajectory to the DMP point
        pt.positions = traj[i]
        # append the new point to the trajectory
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;

#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;

def moveTheRobot(arm, plan):
    jointPositions = plan.plan.points
    nameList = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    
    jointPosList = []
    for temp in jointPositions:
        jointList = []
        for i in range(7):
            jointList.append(temp.positions[i])
        jointPos = dict(zip(nameList, jointList))
        jointPosList.append(jointPos)
    # move the arm follow the generated trajectory
    # parameter: basicTrajMove, the last parameter is frequence, the larger, the faster it sends the waypoints
    arm.basicTrajMove(jointPosList, 0.2, len(jointPosList), 40)
        #arm.basicPositionMove(jointPos, 0.2)

if __name__ == '__main__':
    try:
        rospy.init_node('dmp_massage')
        #Create a DMP from a 7-D joint space trajectory
        # customize some hyper-parameters
        dims = 7
        dt = 0.5 # incremental time
        K = 100 # proportional gain
        D = 2.0 * np.sqrt(K) # derivative gain
        num_bases = 20 # number of gaussian primitives
        inputJoints = [] # store the input training data

        # read training data from file
        with open("listOfJoints.txt", "r") as dataRecorded:
            for line in range(10):
                temp = []
                data = eval(dataRecorded.readline())
                for item in data:
                    temp.append(item)
                print(temp)
                inputJoints.append(temp)

        dataRecorded.close()

        resp = makeLFDRequest(dims, inputJoints, dt, K, D, num_bases)
        makeSetActiveRequest(resp.dmp_list)

        goal_thresh = [0.2,0.2, 0.2, 0.1, 0.1, 0.1, 0.1]
        seg_length = -1          #Plan until convergence to goal
        # parameter: tau, time comparing with demo
        tau = 18 * resp.tau       #Desired plan should take 8 times as long as demo
        dt = 0.2
        integrate_iter = 5       #dt is rather large, so this is > 1  
        #joints_0 = [1.7750703125, -0.4319609375, -0.8911376953125, 1.5901337890625, 0.8923017578125, 0.3399443359375, 0.0406318359375]
        #joints_end = [1.6577998046875, -0.286306640625, -0.954734375, 1.3618857421875, 0.9406533203125, 0.7698525390625, 0.0404248046875]
        joints_0 = [2.1301376953125, -0.3836904296875, -0.9699033203125, 1.8179384765625, 0.98302734375, 0.0798271484375, 0.0437431640625]
        joints_end = [1.901609375, -0.2622255859375, -1.037994140625, 1.5365166015625, 1.1672919921875, 0.8511787109375, -0.169119140625]
        joints_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   
        t_0 = 0                

        plan = makePlanRequest(joints_0, joints_dot_0, t_0, joints_end, goal_thresh, 
                            seg_length, tau, dt, integrate_iter)
        print plan

        # test the dmp on the robot arm
        arm = lowlevelmotion.LowLevelMotion()
        move_speed = 0.2
        moveTheRobot(arm, plan)

    except rospy.ROSInterruptException:
        pass
