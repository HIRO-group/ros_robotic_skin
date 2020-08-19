#!/bin/bash

run_do_calib(){
	for i in "$@"; do
		rosrun imu_calib do_calib "$i"
	done
	echo ""
}

array=()

defnum=0
defimu=0

num_imus=${1:-$defnum}
i_imu=${2:-$defimu}

if [ $num_imus -gt $defnum ]
then
	for i in $(seq 1 $num_imus); do
		#echo "HI"
		array+=($i)
	done
elif [ $i_imu -gt $defimu ]
then
	array+=($i_imu)
fi

run_do_calib "${array[@]}"
