cmake_minimum_required(VERSION 3.0.2)
project(qm_hw)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        qm_common
        qm_msgs
        controller_manager
        urdf
        kinova_driver
        kinova_msgs
        ocs2_core
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        qm_common
        qm_msgs
        controller_manager
        urdf
        kinova_driver
        kinova_msgs
        ocs2_core
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

set(EXTRA_LIBS libunitree_legged_sdk_amd64.so lcm)

add_executable(${PROJECT_NAME}
        src/qm_hw.cpp
        src/QmHW.cpp
        src/QmHWLoop.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        )