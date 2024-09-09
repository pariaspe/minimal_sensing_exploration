# Exploring Unstructured Environments using Minimal Sensing on Cooperative Nano-Drones
If you use this code in your academic work, please cite ([PDF](https://doi.org/10.48550/arXiv.2407.06706)):

```latex
@article{arias2024exploring,
  title={Exploring Unstructured Environments using Minimal Sensing on Cooperative Nano-Drones},
  author={Arias-Perez, Pedro and Gautam, Alvika and Fernandez-Cortizas, Miguel and Perez-Saura, David and Saripalli, Srikanth and Campoy, Pascual},
  journal={arXiv preprint arXiv:2407.06706},
  year={2024}
}
```

This work is released under BSD 3-Clause License.

## Installation
This project has been developed in **Ubuntu 22.04 LTS**, **ROS 2 Humble** and **Aerostack2**.

- Install ROS 2 Humble [[guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)].
- Install Aerostack2 [[guide](https://aerostack2.github.io/_00_getting_started/source_install.html)] forked repository:
```bash
sudo apt install git python3-rosdep python3-pip python3-colcon-common-extensions -y  # previous deps
mkdir -p ~/aerostack2_ws/src/ && cd ~/aerostack2_ws/src/
git clone https://github.com/pariaspe/aerostack2-mininal-sensing-exploration.git
cd ~/aerostack2_ws
sudo rosdep init
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src
```
- Build Aerostack2:
```bash
cd ~/aerostack2_ws
colcon build --symlink-install
```
- Clone launching repository:
```bash
git clone https://github.com/pariaspe/minimal_sensing_exploration.git
```

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

### Launch exploration
```bash
python explore.py
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