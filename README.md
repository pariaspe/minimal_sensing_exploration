# cf_exploration
Crazyflie area exploration.

## How to launch
### Mocap setup
```bash
source ~/mocap_ws/install/setup.bash
tmuxinator start -n mocap -p tmuxinator/mocap.yml
```

### Launch Aerostack2
```bash
# Try ./launch_as2.sh -h for help
./launch_as2.bash -e mocap_pose -r -t
```

## How to stop
```bash
./stop.bash
```

---

## How to choose number of drones
At file `config/swarm_config.yaml` change the list of drones.
```yml
cf0:
  uri: radio://0/80/2M/E7E7E7E7E7
cf1:
  uri: radio://0/80/2M/E7E7E7E700
# Commented drones will not be launched
# cf2:
#   uri: radio://0/80/2M/E7E7E7E702
```