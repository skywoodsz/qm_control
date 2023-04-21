## Usage

### WBC Only

Launch the simulation with:

```
mon launch qm_gazebo empty_world.launch
```

Load the controller:

```
mon launch qm_wbc_controller load_wbc_controller.launch
```

### Topic

- \leg_states: foot position, velocity and contact states.
- \odom: base odometry