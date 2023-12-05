# cf_exploration
Crazyflie area exploration


## Mocap setup
```
source ~/mocap_ws/install/setup.bash
tmuxinator start -n mocap -p tmuxinator/mocap.yml
ros2 lifecycle set /vicon/mocap_vicon_driver_node activate
```

## Crazyflie setup
```
./launch_as2.sh -e mocap_pose -r -t
```