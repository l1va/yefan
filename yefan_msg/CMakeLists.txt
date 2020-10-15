cmake_minimum_required(VERSION 2.8.3)
project(yefan_msg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  IK.srv
  IKFull.srv
  FK.srv
  Trajectory.srv
  JacobianSpatial.srv
  GeneralizedForces.srv
  InertiaMatrix.srv
  Linearization.srv
  InverseDynamics.srv
  LQRCosts.srv
  LQRGain.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package()
