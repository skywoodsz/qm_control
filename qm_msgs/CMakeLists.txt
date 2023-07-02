cmake_minimum_required(VERSION 3.0.2)
project(qm_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  FILES
    ee_state.msg
    arm_torque.msg
    base_state.msg
    MotorState.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)