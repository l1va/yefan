# Yellow Fanuc repo

3D_R-2000iC_165F_210F_IF_v03 - 3D


Requires GuRoBi (http://www.gurobi.com/) and Drake (https://drake.mit.edu/) to compile
For GuRoBi there is free academic licence (need to register)

Launch files for gazebo located in <b>yefan_gazebo<b> package.

### World with services
`roslaunch yefan_gazebo limits_with_services.launch`
`rosrun yefan_controllers [PD, LQR, Main_QP_Limits]`

### Sensing floor with force-torque sensor
`roslaunch yefan_gazebo sensitive_with_limits_with_services.launch`

### Change trajectory in yefan_trajectory_planner/srv_trajectory_planner.py
