# Name: ros_melodic_docker
# Description: installs controllers for ROS-melodic base in ubuntu bionic environment
#
# VERSION       0.1
#
# Use melodic base image with ubuntu
FROM ros:melodic-ros-base-bionic
MAINTAINER Mike Ivanov, lev_matematik@tut.by
# installing needed controllers
RUN apt-get update && apt-get install -y --no-install-recommends \
                ros-melodic-ros-control \
                ros-melodic-ros-controllers \
                ros-melodic-gazebo-ros \
                ros-melodic-gazebo-ros-control \
                libeigen3-dev \
        && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
        python-catkin-tools \
                python-scipy \
                python-matplotlib \
                python-tk \
        && rm -rf /var/lib/apt/lists/*
