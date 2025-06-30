#!/bin/bash
export ROS_HOSTNAME="localhost"
cd $HOME/catkin_ws && catkin build bota_driver_testing --no-deps -j1 -v --cmake-args -DBUILD_HW_TESTS=ON --catkin-make-args tests
source $HOME/catkin_ws/devel/setup.bash
rostest bota_driver_testing bota_driver_testing_reset_service_async.test && rostest bota_driver_testing bota_driver_testing_reset_service_sync.test && rostest bota_driver_testing bota_driver_testing_zero_measurements_async.test && rostest bota_driver_testing bota_driver_testing_zero_measurements_sync.test && rostest bota_driver_testing bota_driver_testing_ft_offsets_async.test && rostest bota_driver_testing bota_driver_testing_ft_offsets_sync.test 
cd $HOME/.ros && catkin_test_results
if [ $? -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
