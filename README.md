# qm_control

## Overview

The branch considers the influence of the **force** on the end effector and requires a **ft sensor** to measure the force.

## Usage

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
# joy
mon launch qm_controllers joy_teleop.launch
```

## Bugs & Feature Requesityts

This project is still in the early stages of development and we welcome feedback.  Please report bugs and request features using the [Issue Tracker](https://github.com/skywoodsz/qm_control/issues) or Email skywoodszcn@gmail.com

## TODO

- [ ] Solve the singularity problem.
- [ ] Add the foot trajectory plannning.
