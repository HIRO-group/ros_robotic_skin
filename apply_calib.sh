
#!/bin/bash

run_apply_calib(){
        for i in "$@"; do
                rosrun imu_calib apply_calib "$i"
        done
        echo ""
}

array=(1 2 3 4 5 6);
run_apply_calib "${array[@]}"


