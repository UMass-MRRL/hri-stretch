# hri-stretch

## Stretch helpful tips & tricks

### Homing stretch
This step must be done every time stretch is turned on, before anything else. 
```bash
stretch_robot_home.py
```

### Xbox Controller Demo
To run the demo with the xbox controller, first make sure stretch has been home'd. In addition, the deadman switch must be pressed (default: RB (Right Bumper))
```bash
roslaunch hri_stretch teleop_joy.launch
```
#### Controls:
- Deadman switch: RB (Right Bumper)
- Driving base: Left stick
- Lift: Right stick up/down
- Extend arm: Right stick right/left
- Rotate gripper: RT/LT (Right/left trigger)
- Close gripper: A
- Open gripper: B
- Move head: D-pad

### Helpful commands:
Home
```bash
stretch_robot_home.py
```
Stow
```bash
stretch_robot_stow.py
```
System check
```bash
stretch_robot_system_check.py
```
Grasp Object Demo
```bash
roslaunch stretch_demos grasp_object.launch
```
Deliver Demo
```bash
roslaunch stretch_demos handover_object.launch
```

### Files in hri_stretch
In `launch/`:
- `all_demos.launch`: An attempt to get the grasp_object demo working with our xbox controller teleop code. The demo does not currently work, but the teleop code does. Also includes mapping.
- `mapping.launch`: Proof of concept file to see how octomap_server works with stretch. Does nothing but publish the server to rviz
- `stretch_driver.launch`: Modified version of stretch_ros `stretch_driver.launch`, which uses our version of the `stretch_driver`
- `teleop_joy.launch`: Launch file to control stretch with the joystick. 

In `nodes/`: 
- `point_publish.py`: Proof of concept node for publishing a point in the same way rviz does. 
- `stretch_driver`: A modified version of stretch_ros `stretch_driver`. This version includes controlling all of stretch's different mechanism. 
- `stretch_driver_demos`: A modified version of `stretch_driver` that also includes calling `grasp_object` with the START button. Does not work very reliably.
The rest of the files in this folder are helper files left so that `stretch_driver` and the launch files work correctly. They have not been modified.