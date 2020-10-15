cmake_minimum_required(VERSION 3.5.1)
project(yefan_gazebo)

add_compile_options(-std=c++11)

set(drake_DIR "/opt/drake/lib/cmake/drake")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        drake CONFIG REQUIRED
        yefan_msg
        )

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} )

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

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory drake::drake ${catkin_LIBRARIES} yaml-cpp)

