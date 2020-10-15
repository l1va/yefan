#!/usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import JointState
from pydrake.all import *
from yefan_api.robot import all_joints, JOINT_STATES, get_joint_states
import os
import rospkg


def getK():
    plant = MultibodyPlant()
    parser = Parser(plant=plant)
    rospack = rospkg.RosPack()
    pth = rospack.get_path('yefan_gazebo')
    robot = parser.AddModelFromFile(pth + "/scripts/test.urdf")
    plant.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
    plant.Finalize()

    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = nq + nv
    nu = plant.num_actuators()
    assert (nx, nu) == (12, 6)

    Q = np.identity(nx)
    R = np.identity(nu)
    # N = np.zeros([nx, nu])

    plant_ctx = plant.CreateDefaultContext()

    act_in_port_ind = plant.get_actuation_input_port(robot).get_index()
    plant_ctx.FixInputPort(act_in_port_ind, [0] * plant.num_velocities())
    plant_ctx.FixInputPort(plant.GetInputPort("applied_generalized_force").get_index(), [0] * plant.num_velocities())

    out_port_ind = plant.get_continuous_state_output_port().get_index()
    lin_plant = Linearize(system=plant, context=plant_ctx, input_port_index=act_in_port_ind,
                          output_port_index=out_port_ind, equilibrium_check_tolerance=1e06)

    # reg = LinearQuadraticRegulator(system=lin_plant, Q=Q, R=R, N=N)
    (K, S) = LinearQuadraticRegulator(A=lin_plant.A(), B=lin_plant.B(), Q=Q, R=R)
    # reg = LinearQuadraticRegulator(system=lin_plant, Q=Q, R=R, N=N)

    # print("shape: ", K.shape, S.shape)
    # print(np.linalg.det(S))

    # assert (reg.D() == -K).all()
    return K


def joint_state_callback(K):
    pubs = []
    for j in all_joints:
        pubs.append(j.command_publisher())

    def js_cb(data):
        # print(data)
        x = [pos_desired[i] - data.position[i] for i in range(len(data.position))] + [vel_desired[i] - data.velocity[i]
                                                                                      for i in
                                                                                      range(len(data.velocity))]
        t = np.matmul(K,x)
        print("T: ", t)
        print("t shape: ", t.shape)
        for i in range(len(pubs)):
            pubs[i].publish(t[i])

    return js_cb


pos_desired = [0] * 6
vel_desired = [0] * 6


def main():
    rospy.init_node('lqr', anonymous=True)
    rospy.Subscriber(JOINT_STATES, JointState, joint_state_callback(getK()))
    rospy.spin()


if __name__ == '__main__':
    main()
