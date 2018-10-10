import rospy
import numpy as np
import math
from scipy import interpolate
import matplotlib.pyplot as plt
import time
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
# below are for the force control:
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint, 
    MotionWaypointOptions,
    InteractionOptions
)
import intera_interface
from kitt.msg import Hotword


# The class is for low level control for sawyer requiring minimum intera SDk
class LowLevelMotion(object):

    def __init__(self):
        self.jointNames = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'] # define the arm's joints names
        self.jointAngles = [0, 0, 0, 0, 0, 0, 0] 
        self.actualPos = dict(zip(self.jointNames, self.jointAngles)) # Store the actual joint positions of the arm
        self.positioning_pose = Pose() # Store the position of the end effector under specific force
        self.vibration_force = 20
        self.stop = False
        
    # called by the joint state subscriber
    def joints_callback(self, data):
        # updating the actual arm joint angles by subscribing to the joint_states topic
        self.jointAngles = [data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6], data.position[7]]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))

    # move the robot based on joint command
    def basicPositionMove(self, pos, speed):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        rate = rospy.Rate(10)
        # define the speed publisher
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [pos['right_j0'], pos['right_j1'], pos['right_j2'], pos['right_j3'], pos['right_j4'], pos['right_j5'], pos['right_j6']]
        command.mode = 1
        # customize the value to change speed
        pub_speed_ratio.publish(speed)
        
        control_diff_record = 10.0
        control_diff_temp = 0.0
        threshold = 0.02
        # terminate the control once the arm moved to the desired joint space within the threshold
        while control_diff_record>threshold:
            pub.publish(command)
            for x,y in zip(self.jointAngles, command.position):
                control_diff_temp = abs(x-y) + control_diff_temp
            control_diff_record = control_diff_temp
            control_diff_temp = 0.0
            rate.sleep()

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")

    def basicTrajMove(self, positions, speed, traj_length, speed_rate):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        rate = rospy.Rate(speed_rate)
        # define the speed publisher
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.mode = 1
        # customize the value to change speed
        pub_speed_ratio.publish(speed)
        
        control_diff_record = 10.0
        control_diff_temp = 0.0
        threshold = 0.015

        # terminate the control once the arm moved to the desired joint space within the threshold
        counter = 0
        while control_diff_record>threshold and counter<traj_length-1:
            command.position = [positions[counter]['right_j0'], positions[counter]['right_j1'], positions[counter]['right_j2'], positions[counter]['right_j3'], positions[counter]['right_j4'], positions[counter]['right_j5'], positions[counter]['right_j6']]
            pub.publish(command)
            for x,y in zip(self.jointAngles, command.position):
                control_diff_temp = abs(x-y) + control_diff_temp
            control_diff_record = control_diff_temp
            control_diff_temp = 0.0
            rate.sleep()
            counter = counter+1

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")

    # this function is for moving the lower arm only to fulfill high frequency requirement for some massage patterns
    def lowerArmBasicMove(self, cur_pos, speed, frequency, repetition):
        # define some parameters:
        # frequency: customize this to change the vibration speed, default: 100
        # repetition: one move vibrates repetation times, default: 20

        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        rate = rospy.Rate(frequency)
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)
        # set the joint angles for the first press position
        command1 = JointCommand()
        command1.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command1.position = [cur_pos['right_j0'], cur_pos['right_j1'], cur_pos['right_j2'], cur_pos['right_j3']-0.05, cur_pos['right_j4']-0.05, cur_pos['right_j5']+0.1, cur_pos['right_j6']]
        command1.mode = 1
        # set the joint angles for the second press position
        command2 = JointCommand()
        command2.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command2.position = [cur_pos['right_j0'], cur_pos['right_j1'], cur_pos['right_j2'], cur_pos['right_j3'], cur_pos['right_j4'], cur_pos['right_j5'], cur_pos['right_j6']]
        command2.mode = 1
        # looping to create vibrating motion
        pub_speed_ratio.publish(speed)
        for i in range(0, repetition):
            self.forcePress(self.vibration_force, 0.1)
            #for j in range(0, 10):
            #    pub.publish(command1)
            #    rate.sleep()
            #rate.sleep()
            
            pub.publish(command2)
            time.sleep(0.1)
            rate.sleep()#0.01s

    # move the arm to zero joint position
    def moveToZero(self):
        #rospy.init_node('arm_low_level_control', anonymous=True)
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        command.mode = 1
        pub_speed_ratio.publish(0.2)

        start_time = rospy.get_time()
        end_time = rospy.get_time()
        rospy.loginfo(">>>>>>>>>> moving the arm to zero joint position >>>>>>>>>>>")
        while end_time-start_time<8:
            pub.publish(command)
            end_time = rospy.get_time()

    # move the arm to the specified position with the specifies speed
    def moveToPoint(self, position, speed_ratio=0.2):
        # speed * maxSpeed
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = position[3]
        pose.orientation.y = position[4]
        pose.orientation.z = position[5]
        pose.orientation.w = position[6]
        # call the inverse kinematics function
        self.ik(pose, speed_ratio)

    # inverse kinematics function integrating inverse kinematics service and send the joint angles to basicPositionMove function to publish joint commands
    def ik(self, pose, speed):
        iksvc = rospy.ServiceProxy('ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')

        resp = iksvc(ikreq)
        # execute the joint command if the inverse kinematics is successful
        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo(">>>>>>>>>> moving the arm >>>>>>>>>>")
            self.basicPositionMove(limb_joints, speed)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

    # call the ikservice to convert way point to joint space
    def waypointToJoint(self, pose):
        iksvc = rospy.ServiceProxy('ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')

        resp = iksvc(ikreq)
        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

    # set the interaction option to activate the force control
    def set_interaction_options(self, force):
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_interaction_control_mode([1, 1, 2, 1, 1, 1])
        interaction_options.set_in_endpoint_frame(True)
        # set the force:
        interaction_options.set_force_command([0.0, 0.0, force, 0.0, 0.0, 0.0])
        return interaction_options.to_msg()

    # set the interaction option to inactivate the force control
    def exitForceControl(self):
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(False)
        msg = interaction_options.to_msg()
        pub.publish(msg)

    # press down the end effector with the specified force and duration
    def forcePress(self, force, duration):
        rospy.loginfo(">>>>>>>>>> exerting force >>>>>>>>>>")
        force_msg = self.set_interaction_options(force)
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        #rospy.logerr("publishiung")
        pub.publish(force_msg)
        rospy.sleep(duration)
        self.exitForceControl()

    # configure the robot
    def prePress(self, wayPoint):
        self.moveToPoint(wayPoint, 0.2)
        self.forcePress(40, 0)

    # lift the robot arm after massaging a point
    def lift(self, cur_waypoint, height):
        des_waypoint = [cur_waypoint[0], cur_waypoint[1], cur_waypoint[2]+height, cur_waypoint[3], cur_waypoint[4], cur_waypoint[5], cur_waypoint[6]]
        self.moveToPoint(des_waypoint, 0.2)
    
    # the callback function of the endpoint_state subscriber
    def endpoint_callback(self, data):
        self.positioning_pose = data.pose

    # positioning the end effector according to the desired force
    def positioning_the_endpoint_to_force(self, force):
        rospy.loginfo(">>>>>>>>>> exerting force >>>>>>>>>>")
        force_msg = self.set_interaction_options(force)
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        pub.publish(force_msg)
        sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_callback)
        rospy.sleep(2.5)
        self.exitForceControl()

    # convert roll pitch yaw to quaternion
    def to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(0.5*yaw)
        sy = math.sin(0.5*yaw)
        cr = math.cos(0.5*roll)
        sr = math.sin(0.5*roll)
        cp = math.cos(0.5*pitch)
        sp = math.sin(0.5*pitch)

        qx = cy*sr*cp - sy*cr*sp
        qy = cy*cr*sp + sy*sr*cp
        qz = sy*cr*cp - cy*sr*sp
        qw = cy*cr*cp + sy*sr*sp

        orientation = [qx, qy, qz, qw]
        return orientation

    def displacement(self, time):
        disp = 0.1667*math.pi*math.sin(time)
        return disp

    def moveACircleCalibrated(self, wayPoint, move_speed, tool_length):
        # define some key way points of the circle
        speed_rate = 40
        circle_radius = tool_length*math.sin(math.pi/6)
        x_start_position = self.positioning_pose.position.x
        x_end_position = self.positioning_pose.position.x
        y_start_position = self.positioning_pose.position.y - circle_radius
        y_end_position = self.positioning_pose.position.y + circle_radius
        rolls_start_position = 7*math.pi/6
        rolls_end_position = 5*math.pi/6
        pitch_start_position = 0
        pitch_end_position = 0
        number_of_interpolation = 150
        curr_yaw = 0

        # the first half circle
        x_positions = [x_start_position, x_start_position+circle_radius, x_end_position]
        y_positions = [y_start_position, y_start_position+circle_radius, y_end_position]
        rolls = [rolls_start_position, math.pi, rolls_end_position]
        pitches = [pitch_start_position, math.pi/6, pitch_end_position]

        f = interpolate.interp1d(rolls, pitches, kind='quadratic')
        g = interpolate.interp1d(y_positions, x_positions, kind = 'quadratic')
        y_positions_new = np.linspace(y_start_position, y_end_position, number_of_interpolation)
        rolls_new = np.linspace(rolls_start_position, rolls_end_position, number_of_interpolation)
        i = 0
        circle_points_joints = []
        
        for i in range (0,len(rolls_new)-1):
            end = self.to_quaternion(rolls_new[i], f(rolls_new)[i], curr_yaw)
            pose = Pose()
            pose.position.x = g(y_positions_new)[i]
            pose.position.y = y_positions_new[i]
            pose.position.z = self.positioning_pose.position.z-0.3*circle_radius
            pose.orientation.x = end[0]
            pose.orientation.y = end[1]
            pose.orientation.z = end[2]
            pose.orientation.w = end[3]
            circle_points_joints.append(self.waypointToJoint(pose))
        self.basicTrajMove(circle_points_joints, move_speed, len(rolls_new), speed_rate)

        # the second half circle
        
        x_positions = [x_end_position, x_end_position-circle_radius, x_start_position]
        y_positions = [y_end_position, y_end_position-circle_radius, y_start_position]
        rolls = [rolls_end_position, math.pi, rolls_start_position]
        pitches = [pitch_end_position, -math.pi/6, pitch_start_position]
        f = interpolate.interp1d(rolls, pitches, kind='quadratic')
        g = interpolate.interp1d(y_positions, x_positions, kind = 'quadratic')
        y_positions_new = np.linspace(y_end_position, y_start_position, number_of_interpolation)
        rolls_new = np.linspace(rolls_end_position, rolls_start_position, number_of_interpolation)
        i = 0
        circle_points_joints = []
        
        for i in range (0,len(rolls_new)-1):
            end = self.to_quaternion(rolls_new[i], f(rolls_new)[i], curr_yaw)
            pose = Pose()
            pose.position.x = g(y_positions_new)[i]
            pose.position.y = y_positions_new[i]
            pose.position.z = self.positioning_pose.position.z-0.3*circle_radius
            pose.orientation.x = end[0]
            pose.orientation.y = end[1]
            pose.orientation.z = end[2]
            pose.orientation.w = end[3]
            circle_points_joints.append(self.waypointToJoint(pose))
        self.basicTrajMove(circle_points_joints, move_speed, len(rolls_new), speed_rate)
        

    # below are some modularized massage patterns:

    # normal press pattern with the waypoints, move_speed, and force specified
    def normalPress(self, wayPoints, lift_height, move_speed, press_duration, force):
        for wayPoint in wayPoints:
            self.moveToPoint(wayPoint, move_speed)
            self.forcePress(force, press_duration)
            self.lift(wayPoint, lift_height)

    # slow-speed repeated normal press
    def repeatedPress(self, wayPoints, lift_height, move_speed, repetition, press_duration):
        force  = [20, 20, 20, 20]
        self.moveToPoint(wayPoints[0], move_speed)
        self.forcePress(force[0], press_duration)

        for j in range(4):
            for i in range(repetition):
                self.moveToPoint(wayPoints[j], move_speed)
                self.forcePress(force[j], press_duration)
                self.lift(wayPoints[j], 0)

    # high-frequency, high-speed lower arm padding pattern
    def armVibrate(self, wayPoints, lift_height, move_speed, press_frequency, repetition, wayPoint_pause=[0, 0.6, 0.4, 1, 0, 0, 0], stop_check=lambda:False):
        # start the high speed vibrating massage:
        # self.moveToPoint(wayPoints[0], move_speed)
        # self.positioning_the_endpoint_to_force(10)
        #sub = rospy.Subscriber('/hotword', Hotword, self.stop)
        #if stop_check():
        if self.stop:
            self.moveToPoint(wayPoint_pause)
            return

        for wayPoint in wayPoints:
            #if stop_check():
            if self.stop:
                self.moveToPoint(wayPoint_pause)
                return
            self.moveToPoint(wayPoint, move_speed)
            #if stop_check():
            if self.stop:
                self.moveToPoint(wayPoint_pause)
                return
            # move to the position specified by the desired force
            self.positioning_the_endpoint_to_force(10)
            # convert the waypoint to joint angles
            joint_angle = self.waypointToJoint(self.positioning_pose)
            #if stop_check():
            if self.stop:
                self.moveToPoint(wayPoint_pause)
                return
            # vibrate the lower arm
            self.lowerArmBasicMove(joint_angle, move_speed, press_frequency, repetition)
            # lift up the arm
            self.lift(wayPoint, 0.05)

    def circularMotion(self, wayPoints, lift_height, move_speed, calibrated):
        counter = 0
        repetition = 2
        self.moveToPoint(wayPoints[0], move_speed)
        self.positioning_the_endpoint_to_force(10)
        for wayPoint in wayPoints:
            self.moveToPoint(wayPoint, move_speed)
            # move to the position specified by the desired force
            self.positioning_the_endpoint_to_force(5)
            # move a circle
            if calibrated:
                tool_length = 0.05
                for i in range(0,repetition):
                    self.moveACircleCalibrated(self.positioning_pose, move_speed, tool_length)
                    if not i == repetition-1:
                        self.lift(wayPoints[counter], 0.01)
                        self.positioning_the_endpoint_to_force(5)
                    
            if not calibrated:
                self.moveACircle(self.positioning_pose, move_speed+0.05)
            # lift up the arm
            self.lift(wayPoints[counter], 0.05)
            counter = counter+1
        
    def moveTraj(self, x_start, key_poses_x, x_end, y_start, key_poses_y, y_end, move_speed, z, orientation):
        number_of_interpolation = 600
        speed_rate = 20
        x_waypoints = [x_start]
        y_waypoints = [y_start]
        for key_pose_x in key_poses_x:
            x_waypoints.append(key_pose_x)
        x_waypoints.append(x_end)
        for key_pose_y in key_poses_y:
            y_waypoints.append(key_pose_y)
        y_waypoints.append(y_end)
        f = interpolate.interp1d(y_waypoints, x_waypoints, kind='quadratic')
        y_waypoints_new = np.linspace(y_start, y_end, number_of_interpolation)
        trajectory = []

        for i in range (1,len(y_waypoints_new)-1):
            pose = Pose()
            pose.position.x = f(y_waypoints_new)[i]
            pose.position.y = y_waypoints_new[i]
            pose.position.z = z
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
            trajectory.append(self.waypointToJoint(pose))
        self.basicPositionMove(trajectory[0], move_speed)
	self.basicTrajMove(trajectory, move_speed, len(trajectory)-1, speed_rate)
