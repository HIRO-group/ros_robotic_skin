#!/bin/bash

run_do_calib(){
	for i in "$@"; do
		rosrun imu_calib do_calib "$i"
	done
	echo ""
}

array=(1 2 3 4 5 6);
run_do_calib "${array[@]}"
