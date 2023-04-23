cmake_minimum_required(VERSION 2.8.3)
project(ped_simulation)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
)

catkin_package(
    
)

catkin_install_python(PROGRAMS
    scripts/ros_utils/obstacles_genertate_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)