cmake_minimum_required(VERSION 3.10)
project(qm_wbc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        ocs2_legged_robot
        dynamic_reconfigure
        qm_common
        qm_msgs
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
        cfg/wbcWigeht.cfg
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        ocs2_legged_robot
        qm_common
        qm_msgs
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
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        qpOASES
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


add_executable(angle_test
        test/angle_test.cpp
        )
add_dependencies(angle_test
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(angle_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(angle_test PRIVATE ${OCS2_CXX_FLAGS})
#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

