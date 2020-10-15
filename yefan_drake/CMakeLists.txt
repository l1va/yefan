cmake_minimum_required(VERSION 3.5.1)
project(yefan_drake)

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

add_executable(srv_drake src/srv_drake.cpp)
target_link_libraries(srv_drake drake::drake ${catkin_LIBRARIES} yaml-cpp)

