
#!/bin/bash
sudo ip link set dev enx00133bb25956 up
sudo rmmod ec_generic
sudo rmmod ec_master
sudo modprobe ec_generic
sudo modprobe ec_master main_devices=00:13:3b:b2:59:56
echo 'options ec_master main_devices=00:13:3b:b2:59:56' | sudo tee /etc/modprobe.d/ethercat-master.conf
sudo systemctl daemon-reload
sudo systemctl restart ethercat.service
sudo ethercat master --verbose


        