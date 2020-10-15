#!/usr/bin/env python
import rospy
from yefan_msg.srv import *
from yefan_api import ik
import numpy as np


def calculate_ik(req):
    transformation_matrix = np.reshape(req.T, (4, 4))
    transformation_matrix[0:2, 3] *= 1000
    q = ik.calculate(transformation_matrix)
    return IKResponse(q)


def calculate_ik_full(req, jacobian_spatial_handle):
    transformation_matrix = np.reshape(req.T, (4, 4))
    transformation_matrix[0:2, 3] *= 1000

    spatial_velocity = req.W
    q = ik.calculate(transformation_matrix)

    J_matrix = np.reshape(jacobian_spatial_handle(q).J, (6, 6))
    qdot = np.dot(np.linalg.inv(J_matrix), spatial_velocity)

    return IKFullResponse(q, qdot)


def server():
    rospy.init_node('ik')
    topic_name = rospy.get_param('~topic')
    topic_jacobian_spatial = rospy.get_param('topic_jacobian_spatial')

    srv_jacobian_spatial = rospy.ServiceProxy(topic_jacobian_spatial, JacobianSpatial)

    # s = rospy.Service(topic_name, IK, calculate_ik)
    s = rospy.Service(topic_name,
                      IKFull,
                      lambda req: calculate_ik_full(req, srv_jacobian_spatial))
    print "[YEFAN_IK_SERVICE] launched."
    rospy.spin()


if __name__ == "__main__":
    server()

