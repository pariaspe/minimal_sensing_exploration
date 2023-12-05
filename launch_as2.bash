#!/bin/bash

usage() {
    echo "  options:"
    echo "      -m: multi agent (default: false)"
    echo "      -e: estimator_type, choices: [raw_odometry, mocap_pose]"
    echo "      -r: record rosbag (default: false)"
    echo "      -t: launch keyboard teleoperation (default: false)"
    echo "      -n: drone namespace (default: cf)"
}

# Arg parser
while getopts "e:mrtn:" opt; do
  case ${opt} in
    m )
      swarm="true"
      ;;
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    n )
      drone_namespace="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

source utils/tools.bash

# Shift optional args
shift $((OPTIND -1))

swarm=${swarm:="false"}
estimator_plugin=${estimator_plugin:="raw_odometry"}  # default optical_flow_deck
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
drone_namespace=${drone_namespace:="cf"}

if [[ ${swarm} == "true" ]]; then
  num_drones=3
else
  num_drones=1
fi

# Generate the list of drone namespaces
drone_ns=()
for ((i=0; i<${num_drones}; i++)); do
  drone_ns+=("$drone_namespace$i")
done

for ns in "${drone_ns[@]}"
do
  if [[ ${ns} == ${drone_ns[0]} ]]; then
    base_launch="true"
  else
    base_launch="false"
  fi 

  tmuxinator start -n ${ns} -p tmuxinator/session.yml drone_namespace=${ns} base_launch=${base_launch}  estimator_plugin=${estimator_plugin} &
  wait
done

# if [[ ${estimator_plugin} == "mocap_pose" ]]; then
#   tmuxinator start -n mocap -p tmuxinator/mocap.yml &
#   wait
# fi

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t ${drone_ns[0]}:mission
