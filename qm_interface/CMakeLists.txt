cmake_minimum_required(VERSION 3.10)
project(qm_interface)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        tf
        urdf
        kdl_parser
        robot_state_publisher
        ocs2_legged_robot
        ocs2_self_collision
        ocs2_ros_interfaces
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        tf
        urdf
        kdl_parser
        robot_state_publisher
        ocs2_legged_robot
        ocs2_self_collision
        DEPENDS
        pinocchio
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
        src/QMInterface.cpp
        src/common/ModelSettings.cpp
        src/dynamics/QMDynamicsAD.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/QMPreComputation.cpp
        src/initialization/QMInitializer.cpp
        src/constraint/EndEffectorConstraint.cpp
        src/visualization/qm_visualization.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
