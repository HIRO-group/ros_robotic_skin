#!/bin/bash

run_do_calib(){
	for i in "$@"; do
		rosrun imu_calib do_calib "$i"
	done
	echo ""
}

array=()
read -p "How many IMUs would you like to calibrate: " imu_num
echo "Number of IMUs: $imu_num"

for i in $(seq 1 $imu_num); do
	#echo "HI"
	read -p "Enter the imu number: " num
	echo "Entered imu number: $num "
	array+=($num)
done
run_do_calib "${array[@]}"
