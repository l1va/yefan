#!/usr/bin/env python
import rospy
from yefan_msg.srv import *
import numpy as np
from math import pi, sin, cos
import random


def get_square_trajectory_point(t):
    period = 10.0
    square_size = 1.0

    side_period = period / 4
    frequency = 2 * pi / period
    r = square_size / 2

    t = t % period

    x, y, z = 0.0, 0.0, 0.0
    vx, vy, vz = 0.0, 0.0, 0.0

    if 0 <= t < side_period:
        dt = (t - 0.0) / side_period
        x = r * (1 - 2*dt)
        z = r
        vx = -2.0 * r / side_period
    if 1.0*side_period <= t < 2.0*side_period:
        dt = (t - 1.0*side_period) / side_period
        x = -r
        z = r * (1 - 2.0*dt)
        vz = -2.0 * r / side_period
    if 2.0*side_period <= t < 3.0 * side_period:
        dt = (t - 2.0*side_period) / side_period
        x = -r * (1 - 2*dt)
        z = -r
        vx = 2.0 * r / side_period
    if 3.0*side_period <= t < 4.0*side_period:
        dt = (t - 3.0*side_period) / side_period
        x = r
        z = -r * (1 - 2.0*dt)
        vz = 2.0 * r / side_period

    #z += r + 0.2

    x = x + 1.807 - r
    y = y + 0.0 - 0.2
    z = z + 1.97 - r
    
    transformation_matrix = [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ]

    spatial_velocity = [0.0, 0.0, 0.0, vx, vy, vz]

    return transformation_matrix, spatial_velocity


def get_sensing_trajectory_point(t):
    x = prev_point.x
    y = prev_point.y
    z = prev_point.z

    period = 3.0
    # 3-stage schedule
    if t % period < period/3:
        # setting EE position
        z = 0.1
        if not prev_point.updated:
            x = 1.5 + random.uniform(-0.4, 0.4)
            y = 0.0 + random.uniform(-1.0, 1.0)
            prev_point.updated = True
    elif t % period < 2*period/3:
        # moving EE to the ground
        z = 0.0
    else:
        # moving EE from the ground
        z = 0.1
        prev_point.updated = False

    prev_point.x = x
    prev_point.y = y
    prev_point.z = z

    y_angle = -pi/2
    transformation_matrix = [
        [cos(y_angle),  0.0, -sin(y_angle), x],
        [0.0,           1.0, 0.0,           y],
        [sin(y_angle),  0.0, cos(y_angle),  z],
        [0.0,           0.0, 0.0,           1.0]
    ]

    spatial_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return transformation_matrix, spatial_velocity


class CartesianTrajectoryPoint:
    def __init__(self, x, y, z, updated):
        self.x = x
        self.y = y
        self.z = z
        self.updated = updated


prev_point = CartesianTrajectoryPoint(1.5, 0, 1.0, False)


def trajectory_callback(req):
    time = req.t
    # transformation_matrix, spatial_velocity = get_square_trajectory_point(time)
    transformation_matrix, spatial_velocity = get_sensing_trajectory_point(time)
    return TrajectoryResponse(
        np.ravel(transformation_matrix),
        spatial_velocity
    )


def server():
    rospy.init_node('traj')
    topic_name = rospy.get_param('/topic_trajectory_planner')
    s = rospy.Service(topic_name, Trajectory, trajectory_callback)
    print "[YEFAN_TRAJECTORY_PLANNER_SERVICE] launched."
    rospy.spin()


if __name__ == "__main__":
    server()
