cmake_minimum_required(VERSION 3.0.2)
project(qm_wbc_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        qm_common
        qm_interface
        qm_estimation
        qm_wbc
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        dynamic_reconfigure
        qm_msgs
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        qm_common
        qm_interface
        qm_estimation
        controller_interface
        ocs2_legged_robot_ros
        angles
        dynamic_reconfigure
        qm_msgs
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        DEPENDS
        pinocchio
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/QMWbcController.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
