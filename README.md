# qm_control

## Overview

The branch considers **force disturbance** on the manipulator's end-effector and requires an **ft sensor** to measure the force.

<div align=center><img src="./docs/force.gif"></div>

## Usage

Launch the simulation with:

```
mon launch qm_gazebo empty_world.launch
```

Load the controller:

```
mon launch qm_controllers load_controller.launch
```

Start the controller using `rqt_controller_manager` GUI:

```
rosrun rqt_controller_manager rqt_controller_manager
```

After the manipulator is initialized, commands can be sent as:

```
# Don't use mon
roslaunch qm_controllers load_qm_target.launch 
# rviz
mon launch qm_controllers rviz.launch
```

## External force perception

```
rostopic echo /ft_sensor_topic
```

## External force test

You can call the service to apply external force $[30.0, 0.0, 0.0]^T$ to the end-effector for 3 seconds.

```
rosservice call /gazebo/apply_body_wrench '{body_name: "qm::ft_sensor", reference_frame: "qm::ft_sensor", wrench: { force: { x: 30.0, y: 0, z: 0 } }, start_time: 0, duration: {secs: 3} }'
```

## Bugs & Feature Requests

This project is still in the early stages of development and we welcome feedback.  Please report bugs and request features using the [Issue Tracker](https://github.com/skywoodsz/qm_control/issues) or Email skywoodszcn@gmail.com
