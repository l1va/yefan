cmake_minimum_required(VERSION 3.5.1)
project(yefan_gazebo)

add_compile_options(-std=c++11)

set(drake_DIR "/opt/drake/lib/cmake/drake")
set(GUROBI_DIR "/opt/gurobi811/linux64/include")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        drake CONFIG REQUIRED
        yefan_msg
        )

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(include "/opt/gurobi811/linux64/include" )

catkin_package(
        LIBRARIES
        DEPENDS
        roscpp
        roslib
        drake
        yefan_msg
)

add_executable(lqr src/lqr.cpp)
target_link_libraries(lqr drake::drake ${catkin_LIBRARIES} yaml-cpp)

add_executable(cartesian_trajectory src/cartesian_trajectory.cpp)
target_link_libraries(cartesian_trajectory drake::drake ${catkin_LIBRARIES} yaml-cpp)

add_executable(visualisation-test src/visualisation-test.cpp)
target_link_libraries(visualisation-test ${catkin_LIBRARIES})

add_executable(PD src/PD.cpp)
target_link_libraries(PD drake::drake ${catkin_LIBRARIES} yaml-cpp)

add_executable(QP src/QP.cpp)
target_link_libraries(QP drake::drake ${catkin_LIBRARIES} yaml-cpp -L/opt/gurobi811/linux64/lib/ -lgurobi_g++5.2 -L/opt/gurobi811/linux64/lib/ -lgurobi81)

add_executable(QP_strict_limits src/QP_strict_limits.cpp)
target_link_libraries(QP_strict_limits drake::drake ${catkin_LIBRARIES} yaml-cpp -L/opt/gurobi811/linux64/lib/ -lgurobi_g++5.2 -L/opt/gurobi811/linux64/lib/ -lgurobi81)

add_executable(QP_velocity_limits src/QP_velocity_limits.cpp)
target_link_libraries(QP_velocity_limits drake::drake ${catkin_LIBRARIES} yaml-cpp -L/opt/gurobi811/linux64/lib/ -lgurobi_g++5.2 -L/opt/gurobi811/linux64/lib/ -lgurobi81)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory drake::drake ${catkin_LIBRARIES} yaml-cpp)
