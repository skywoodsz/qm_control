# qm_control

<table><tr>
<td><img src="./docs/position_cmd.gif"  width = "400"/></td>
<td><img src="./docs/chicken_hand.gif"  width = "400"/></td>
</tr></table> 
<table><tr>
<td><img src="./docs/202306191701 00_00_33-00_00_48.gif" width = "400" /></td>
<td><img src="./docs/202306291554 00_01_03-00_01_19~2.gif" width = "400"  /></td>
</tr></table> 

***Video Links:*** [YouTube](https://youtu.be/JCn5obOh4D8), [Bilibili](https://www.bilibili.com/video/BV1uP411v7Ab) (for China).

## Overview

qm_control is a controller for quadruped manipulators using model predictive control and whole-body control. The controller aims to make the quadruped manipulator more athletic. It has achieved tasks such as *whole-body planning*, *end-effector motion tracking*, *stability with force disturbance*, and *whole-body compliance control*. We will continue to develop more features for the controller. **The project is still under development, not the final version**.

***Notes:*** Four branches focus on different tasks.

-   The main branch focuses on the whole-body motion and assumes no force acting on the manipulator's end-effector.
-   [feature-force](https://github.com/skywoodsz/qm_control/tree/feature-force) branch focuses on keeping stability with force disturbance acting on the manipulator's end-effector.
-   [feature-compliance](https://github.com/skywoodsz/qm_control/tree/feature-compliance) branch focuses on whole-body compliance control.
-   [feature-real](https://github.com/skywoodsz/qm_control/tree/feature-real) branch implements the controller on hardware.

## Related Paper  
[1] 张天霖. 基于视觉伺服与集值反馈的四足机械臂动态物体抓取研究 [D]. 哈尔滨: 哈尔滨工业大学, 2024. (for China).  
*Notes: 可以通过**哈尔滨工业大学图书馆**和**深圳大学城图书馆**搜索查询。*

[2] To be soon.

## Installation

### Install dependencies

- [OCS2](https://leggedrobotics.github.io/ocs2/installation.html#prerequisites)
- [ROS1-Noetic](http://wiki.ros.org/noetic)

### Clone and Build

```
# Clone
mkdir -p <catkin_ws_name>/src
cd <catkin_ws_name>/src
git clone https://github.com/skywoodsz/qm_control.git

# Build
cd <catkin_ws_name>
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
```

***Notes: Make sure OCS2 is in the environment path.***

## Usage

We have two versions of the controller:  **MPC with WBC** and **only MPC**.

### MPC-WBC

Launch the simulation with:

```
mon launch qm_gazebo empty_world.launch
```

Load the controller:

```
mon launch qm_controllers load_controller.launch
```

Start the controller using `rqt_controller_manager` GUI

```
rosrun rqt_controller_manager rqt_controller_manager
```

After the manipulator is initialized, commands can be sent

```
# Don't use mon
roslaunch qm_controllers load_qm_target.launch 
# rviz
mon launch qm_controllers rviz.launch
```

### MPC Only

Launch the simulation with:

```
mon launch qm_gazebo empty_world_mpc.launch
```

Load the controller:

```
mon launch qm_controllers load_controller_mpc.launch
```

## Gamepad Control

You can use the gamepad to control the quadruped base and the manipulator's end-effector, respectively. The schematic diagram of the gamepad is as follows:
<p align = "center">
<img src="./docs/gamepad.png" alt="gamepad" width = "500"/>
</p>

## End-effector stability testing in simulation

<table><tr>
<td><img src="./docs/position_err.png"  width = "400"/></td>
<td><img src="./docs/angle_err.png"  width = "400"/></td>
</tr></table> 

***Analysis***: The motion of the base and end-effector pose w.r.t. the initial pose when the end-effector is controlled to remain at a fixed pose during locomotion. While the base travels 30 cm, the end-effector’s deviation from its initial position is at most 3.5 mm and 2.6 degrees.

## Bugs & Feature Requests

The project is still in the early stages of development and we welcome feedback.  Please report bugs and request features using the [Issue Tracker](https://github.com/skywoodsz/qm_control/issues) or Email skywoodszcn@gmail.com

## TODO

- [ ] Solve the singularity problem.
- [ ] Add the foot trajectory planning.
- [ ] Merge the branch feature-real to the main.
