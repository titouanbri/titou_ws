#!/bin/bash
sudo ip link set dev enxa0cec89e23c0 up
sudo rmmod ec_generic
sudo rmmod ec_master
sudo modprobe ec_generic
sudo modprobe ec_master main_devices=a0:ce:c8:9e:23:c0
echo 'options ec_master main_devices=a0:ce:c8:9e:23:c0' | sudo tee /etc/modprobe.d/ethercat-master.conf
sudo systemctl daemon-reload
sudo systemctl restart ethercat.service
sudo ethercat master --verbose