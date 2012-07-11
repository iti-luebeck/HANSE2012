#!/bin/bash
if [ $UID != 0 ]; then
	exec sudo bash $0
fi

IF=wlan1

echo "Unloading iwlwifi, error msg about /proc/modules is OK"
rmmod iwlwifi

echo "Stopping network manager, error about unknown instance is OK"
service network-manager stop

echo "Setting interface settings"
ifconfig $IF up 10.13.37.2/24

echo "Starting wpa-supplicant"
sleep 2
sudo wpa_supplicant -Dwext -i$IF -c wpa.conf -d
