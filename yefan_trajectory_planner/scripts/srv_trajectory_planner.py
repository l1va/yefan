#!/usr/bin/env python
import rospy
from yefan_msg.srv import *
import numpy as np
from math import pi


def get_square_trajectory_point(t):
    period = 8.0
    square_size = 2.0

    side_period = period / 4
    frequency = 2 * pi / period
    r = square_size / 2

    t = t % period

    x, y, z = 1.5, 0.0, 0.0
    vx, vy, vz = 0.0, 0.0, 0.0

    if 0 <= t < side_period:
        dt = (t - 0.0) / side_period
        y = r * (1 - 2*dt)
        z = r
        vy = -2.0 * r / side_period
    if 1.0*side_period <= t < 2.0*side_period:
        dt = (t - 1.0*side_period) / side_period
        y = -r
        z = r * (1 - 2.0*dt)
        vz = -2.0 * r  / side_period
    if 2.0*side_period <= t < 3.0 * side_period:
        dt = (t - 2.0*side_period) / side_period
        y = -r * (1 - 2*dt)
        z = -r
        vy = 2.0 * r / side_period
    if 3.0*side_period <= t < 4.0*side_period:
        dt = (t - 3.0*side_period) / side_period
        y = r
        z = -r * (1 - 2.0*dt)
        vz = 2.0 * r / side_period

    transformation_matrix = [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ]

    y += square_size/2 + 0.2

    spatial_velocity = [0.0, 0.0, 0.0, vx, vy, vz]

    return transformation_matrix, spatial_velocity


def trajectory_callback(req):
    time = req.t
    transformation_matrix, spatial_velocity = get_square_trajectory_point(time)
    return TrajectoryResponse(
        np.ravel(transformation_matrix),
        spatial_velocity
    )


def server():
    rospy.init_node('traj')
    topic_name = rospy.get_param('~topic')
    s = rospy.Service(topic_name, Trajectory, trajectory_callback)
    print "[YEFAN_TRAJECTORY_PLANNER_SERVICE] launched."
    rospy.spin()


if __name__ == "__main__":
    server()
