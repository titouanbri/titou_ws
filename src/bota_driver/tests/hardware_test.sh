#!/bin/bash
export ROS_HOSTNAME="localhost"
cd $HOME/catkin_ws && catkin build bota_driver_testing --no-deps -j1 -v --cmake-args -DBUILD_HW_TESTS=ON --catkin-make-args run_tests && catkin_test_results

if [ $? -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 0
fi
