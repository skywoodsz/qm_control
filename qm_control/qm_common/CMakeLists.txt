cmake_minimum_required(VERSION 3.10)
project(qm_common)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        hardware_interface
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        hardware_interface
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE include)