# qm_control

## Overview

The branch is the implement on the real robot hardware.

## Usage

Launch the real robot hardware with:

```
# must
sudo su

# source it
source <catkin_ws_name>/devel/setup.bash

# Start the hardware
mon launch qm_hw qm_hw.launch
```

After the manipulator is initialized, the controller can be loaded:

```
mon launch qm_controllers load_controller.launch
```

Start the controller using `rqt_controller_manager` GUI

```
rosrun rqt_controller_manager rqt_controller_manager
```

Send the command

```
# Don't use mon
roslaunch qm_controllers load_qm_target.launch 
# rviz
mon launch qm_controllers rviz.launch
```

## Bugs & Feature Requesityts

This project is still in the early stages of development and we welcome feedback.  Please report bugs and request features using the [Issue Tracker](https://github.com/skywoodsz/qm_control/issues) or Email skywoodszcn@gmail.com

## TODO

- [ ] Solve the singularity problem.
- [ ] Add the foot trajectory plannning.
- [ ] Merge branch feature-real to main.
