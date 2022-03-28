# exit if something goes wrong
#set -e
# list the actual files
ls -lha
# remove running kernel
sudo rmmod v-hub
# unload the overlay
sudo dtoverlay -r 0
# copy kernel to lib
sudo cp ./v-hub.ko /lib/modules/$(uname -r)/extras
# load the kernel
sudo modprobe v-hub
# copy the overlay
sudo dtoverlay vhub-overlay.dtbo
# print the last 10 kernel prints
dmesg  | tail -n 10
# list the loaded overlays
# you can look: cd /sys/kernel/config/device-tree/overlays/
sudo dtoverlay -l
# list the gpio, for help raspi-gpio help
 #raspi-gpio get 5,25

 # adding the gadget descriptors
 # cdc serial
 sudo ./gadget/cdc-acm.sh
 # cdc ethernet
 sudo ./gadget/cdc-ecm.sh up
