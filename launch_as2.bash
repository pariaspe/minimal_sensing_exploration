#!/bin/bash

usage() {
    echo "  options:"
    echo "      -e: estimator_type, choices: [raw_odometry, mocap_pose]"
    echo "      -r: record rosbag (default: false)"
    echo "      -t: launch keyboard teleoperation (default: false)"
}

# Arg parser
while getopts "e:rt" opt; do
  case ${opt} in
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[rt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Shift optional args
shift $((OPTIND -1))

estimator_plugin=${estimator_plugin:="raw_odometry"}  # default optical_flow_deck
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

# Get drone namespaces from swarm config file
drones=$(python utils/get_drones.py config/swarm_config.yaml)

# if [[ ${estimator_plugin} == "mocap_pose" ]]; then
#   tmuxinator start -n mocap -p tmuxinator/mocap.yml &
#   wait
# fi

tmuxinator start -n aerostack2 -p tmuxinator/aerostack2.yml drones=${drones} estimator_plugin=${estimator_plugin} &
wait

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml drone_namespace=${drones} &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  # TODO: Keyboard Teleop uses ',' as separator for drone namespaces
  drones_sep=$(python utils/get_drones.py config/swarm_config.yaml --sep ",")
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml drone_namespace=${drones_sep} &
  wait
fi

# Attach to tmux session aerostack2, window 0
tmux attach-session -t aerostack2:alphanumeric_viewer
