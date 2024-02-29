cmake_minimum_required(VERSION 3.10)
project(qm_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        qm_common
        gazebo_dev
        gazebo_ros_control
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        qm_hw_sim
        CATKIN_DEPENDS
        roscpp
        qm_common
        gazebo_ros_control
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(qm_hw_sim
        src/QMHWSim.cpp
        )

add_dependencies(qm_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(qm_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )


