#!/bin/bash

# This function will loop and run do_calib on each imu
run_do_calib(){
	for i in "$@"; do
		rosrun imu_calib do_calib "$i"
	done
	echo ""
}

# Array that will hold the imus the user wants to calibrate
array=()

# Default value for the number of imus
defnum=0

# Default value for the specific imu number 
defimu=0

# Sets the default value to the 
# num_imus and i_imu (which is the specific imu to be calibrated) values
num_imus=${1:-$defnum}
i_imu=${2:-$defimu}

# If the user has inputted a value for the num_imus argument
# Then, the value will be greater than the default
# Allowing the array to fill with numbers from 1 to num_imus (inclusive)
if [ $num_imus -gt $defnum ]
then
	for i in $(seq 1 $num_imus); do
		array+=($i)
	done

# If the user has not inputted a value for the num_imus argument
# But has inputted a value for the i_imu argument
# Then add that single value to the array
elif [ $i_imu -gt $defimu ]
then
	array+=($i_imu)
fi

# Then run run_do_calib with the array of imus
run_do_calib "${array[@]}"
