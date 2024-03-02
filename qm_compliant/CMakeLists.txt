cmake_minimum_required(VERSION 3.10)
project(qm_compliant)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        dynamic_reconfigure
        ocs2_legged_robot
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
        cfg/impemdance.cfg
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        ocs2_legged_robot
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
        src/CompliantBase.cpp
        src/CartesianImpendance.cpp
        src/BoundedAdmittance.cpp
        src/Admittance.cpp
        src/BaseAdmittance.cpp
        src/BoundedAdmittanceWithK.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )