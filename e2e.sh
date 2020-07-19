#!/bin/bash

# HIRO Group end-to-end (e2e) script
# for running data collection and calibration for roboskin.

# go through specific arguments
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    --roboskin-path)
    ROBOSKIN_PATH="$2"
    shift # past argument
    shift # past value
    ;;
    --experiment-type)
    if [[ "$2" == "real" || "$2" == "sim" ]]
    then
        EXPERIMENT_TYPE="$2"
    else
        echo "sim or real not selected. Resorting to sim option."
        EXPERIMENT_TYPE="sim"
    fi
    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    shift # past argument
    ;;
esac
done

# input validation
if [[ -z "$ROBOSKIN_PATH" ]]
then
    echo "--roboskin-path was not set. Exiting now..."
    exit 1
fi

if [[ ! -d "$ROBOSKIN_PATH" ]]
then
    echo "--roboskin-path was not found. Exiting now..."
    exit 1
fi

if [[ ! -f "$ROBOSKIN_PATH/robotic_skin/examples/calibration/calibrate_imu_poses.py" ]]
then
    echo "--roboskin-path does not contain robotic_skin/examples/calibration/calibrate_imu_poses.py. Exiting now..."
    exit 1
fi

# execution of data collection
if [[ "$EXPERIMENT_TYPE" == "sim" ]]
then
    roslaunch ros_robotic_skin e2e_panda_sim.launch
else
    roslaunch ros_robotic_skin e2e_panda_real.launch
fi

# sleep to wait for complete shutdown of ros nodes
sleep 2

# execution of optimization
python3 $ROBOSKIN_PATH/robotic_skin/examples/calibration/calibrate_imu_poses.py --log INFO

sleep 2

roslaunch ros_robotic_skin visualization_imus.launch