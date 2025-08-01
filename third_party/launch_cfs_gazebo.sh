#!/bin/bash
function cleanup() {
	pkill -x cf2
	pkill -9 ruby
}

function spawn_model() {
	MODEL=$1
	N=$2 # Cf ID
	X=$3 # spawn x position
	Y=$4 # spawn y position
	X=${X:=$X}
	Y=${Y:=$Y}
	SUPPORTED_MODELS=("crazyflie", "crazyflie_thrust_upgrade")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi
	
	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null


	set --
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/models/${MODEL}/model.sdf.jinja
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo
	set -- ${@} --cffirm_udp_port $((19950+${N}))
	set -- ${@} --cflib_udp_port $((19850+${N}))
	set -- ${@} --cf_id $((${N}))
	set -- ${@} --cf_name cf
	set -- ${@} --output-file /tmp/${MODEL}_${N}.sdf

	python3 ${@}

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

    gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: 0.5}}, name: "'${MODEL}_${N}'", allow_renaming: 1'
	
	echo "starting instance $N in $(pwd)"
	$build_path/cf2 $((19950+${N})) > out.log 2> error.log &

	popd &>/dev/null
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Description: This script is used to spawn multiple vehicles in gazebo from the crazyflies.yaml file in crazyswarm2/crazyflie/config."
	echo "Usage: $0 [-m <vehicle_model>] [-w <world>]"
	exit 1
fi

while getopts m:w:f: option
do
	case "${option}"
	in
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		f) COORDINATES_FILE=${OPTARG};;
	esac
done

world=${WORLD:=crazysim_default}
vehicle_model=${VEHICLE_MODEL:="crazyflie"}
coordinates_file=${COORDINATES_FILE:="crazyflies.yaml"}
export CF2_SIM_MODEL=gz_${vehicle_model}


# Find CrazySim package dynamically (skip .git folders)
crazysim_path=""
while IFS= read -r -d '' path; do
    # Skip if path contains .git
    if [[ "$path" != *".git"* ]]; then
        crazysim_path="$path"
        break
    fi
done < <(find "/mounted_volume" -name "CrazySim" -type d -print0 2>/dev/null)

if [ -z "$crazysim_path" ] || [ ! -d "$crazysim_path" ]; then
    echo "ERROR: Could not find CrazySim package (excluding .git folders)"
    echo "Searched in: /mounted_volume"
    exit 1
fi

echo "Found CrazySim at: $crazysim_path"
src_path=${crazysim_path}/crazyflie-firmware
build_path=${src_path}/sitl_make/build

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true

sleep 1

source ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash ${src_path} ${build_path}

echo "Starting gazebo"
gz sim -s -r ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/worlds/${world}.sdf -v 3 &
sleep 3

n=0

# Find the crazyswarm2 package location dynamically

crazyswarm2_path=$(find "/mounted_volume" -name "crazyswarm2" -type d 2>/dev/null | head -1)

echo "Found crazyswarm2 at: $crazyswarm2_path"

# Set the config file path
config_file="${crazyswarm2_path}/crazyflie/config/${coordinates_file}"

# Parse YAML to find robots section and extract enabled robots
in_robots_section=false
current_robot_name=""
while IFS= read -r line; do
    # Skip commented lines and empty lines
    if [[ $line =~ ^[[:space:]]*# ]] || [[ -z "${line// }" ]]; then
        continue
    fi
    
    # Check if we're entering the robots section
    if [[ $line =~ ^[[:space:]]*robots:[[:space:]]*$ ]]; then
        in_robots_section=true
        echo "In robots section"
        continue
    fi
    if [[ $line =~ ^[[:space:]]*(cf_[0-9]+): ]]; then
        current_robot_name=${BASH_REMATCH[1]}
        echo "Current robot name: $current_robot_name"
    fi
    # Check if we're leaving the robots section (next top-level key)
    if [[ $in_robots_section == true ]] && [[ $line =~ ^[^[:space:]] ]]; then
        in_robots_section=false
        echo "Leaving robots section"
        continue
    fi
    
    # Only process lines within the robots section
    if [[ $in_robots_section == true ]]; then
        # Check if this is a robot entry with enabled: true
        # echo "Processing line: $line"
        if [[ $line =~ initial_position:[[:space:]]*\[ ]]; then
            # Extract [x, y, z] values directly from this line
            coords=$(echo "$line" | sed 's/.*initial_position:[[:space:]]*\[\([^]]*\)\].*/\1/')
            x=$(echo "$coords" | cut -d',' -f1 | tr -d ' ')
            y=$(echo "$coords" | cut -d',' -f2 | tr -d ' ')
            spawn_model ${vehicle_model} $(($n)) $x $y
            n=$(($n + 1))
        fi
    fi
done < "$config_file"


trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo gui"
# gdb ruby
gz sim -g