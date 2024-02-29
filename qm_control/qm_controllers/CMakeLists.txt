cmake_minimum_required(VERSION 3.10)
project(qm_controllers)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        qm_common
        qm_interface
        qm_estimation
        qm_wbc
        qm_msgs
        controller_interface
        ocs2_legged_robot_ros
        angles
        dynamic_reconfigure
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
      cfg/weight.cfg
)    

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
            qm_wbc
            qm_msgs
            controller_interface
            ocs2_legged_robot_ros
            angles
            dynamic_reconfigure
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

# Declare a cpp library
add_library(${PROJECT_NAME}
        src/QMController.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


###########
##  EXE  ##
###########
add_executable(qm_target_trajectories_publisher
     src/QmTargetTrajectoriesPublisher_node.cpp
     src/QmTargetTrajectoriesPublisher.cpp
     src/GaitJoyPublisher.cpp
     )
add_dependencies(qm_target_trajectories_publisher
     ${catkin_EXPORTED_TARGETS}
     )
target_link_libraries(qm_target_trajectories_publisher
     ${catkin_LIBRARIES}
     )
target_compile_options(qm_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})






