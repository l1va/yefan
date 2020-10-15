#!/usr/bin/env python
import rospy
from yefan_msg.srv import *
import fk
import numpy as np


def calculate_fk(req):
    transformation_matrix = fk.calculate(req.q)
    transformation_matrix[0:3, 3] /= 1000
    return FKResponse(np.ravel(transformation_matrix))


def server():
    rospy.init_node('fk')
    topic_name = rospy.get_param('/topic_forward_kinematics')

    s = rospy.Service(topic_name, FK, calculate_fk)
    print "[YEFAN_FK_SERVICE] launched."
    rospy.spin()


if __name__ == "__main__":
    server()
