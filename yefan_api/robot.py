#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

joint1_Joint = 'joint1'
joint2_Joint = 'joint2'
joint3_Joint = 'joint3'
joint4_Joint = 'joint4'
joint5_Joint = 'joint5'
joint6_Joint = 'joint6'

d0 = 346
d11 = 324
d12 = 312
d2 = 1075
d31 = 225
#d32 do not know, added to d4:
d4 = 1280
d5 = 215

YEFAN_PREFIX = "yefan"
JOINT_STATES = '/' + YEFAN_PREFIX + "/joint_states"


class JointStatesWrapper:
    def __init__(self, js_msg):
        self.js_msg = js_msg
        self.names_map = {}
        for i in range(len(js_msg.name)):
            self.names_map[js_msg.name[i]] = i

    def position(self, joint):
        if joint not in self.names_map:
            raise KeyError("joint (" + joint + ") not found in names: " + str(self.js_msg.name))
        return self.js_msg.position[self.names_map[joint]]


def get_joint_states():
    return JointStatesWrapper(rospy.wait_for_message(JOINT_STATES, JointState, timeout=4))


def joint_topic(joint, command, suffix=""):
    return "/" + YEFAN_PREFIX + "/" + joint + suffix + "/" + command


def joint_command_topic(joint):
    return joint_topic(joint, "command", "_position_controller")


class JointDevice:
    def __init__(self, joint, min_pos, max_pos):
        self.joint = joint
        self.min_pos = min_pos
        self.max_pos = max_pos

    def command_publisher(self):
        return rospy.Publisher(joint_command_topic(self.joint), Float64, queue_size=10)


joint1 = JointDevice(joint=joint1_Joint, min_pos=-3.14, max_pos=3.14) # from -185 to 185 (370)
joint2 = JointDevice(joint=joint2_Joint, min_pos=-1.0472, max_pos=1.32645) # from -60 to 76 (136)
joint3 = JointDevice(joint=joint3_Joint, min_pos=-1.8, max_pos=1.8) # from -80 to 180? (312)
joint4 = JointDevice(joint=joint4_Joint, min_pos=-3.14, max_pos=3.14) # from -360 to 360 (720)
joint5 = JointDevice(joint=joint5_Joint, min_pos=-2.18166, max_pos=2.18166) # from -125 to 125 (250)
joint6 = JointDevice(joint=joint6_Joint, min_pos=-3.14, max_pos=3.14) # from -360 to 360 (720)

all_joints = [joint1, joint2, joint3, joint4, joint5, joint6]
