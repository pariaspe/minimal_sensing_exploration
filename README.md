# cf_exploration
Crazyflie area exploration

## Mocap setup
```
source ~/mocap_ws/install/setup.bash
tmuxinator start -n mocap -p tmuxinator/mocap.yml
```

## Crazyflie setup
```
./launch_as2.sh -e mocap_pose -r -t
```