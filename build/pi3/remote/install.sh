# list the actual files
ls -lha
# remove running kernel
sudo rmmod v-hub
# unload the overlay
sudo dtoverlay -r 0
# copy kernel to lib
sudo cp ./v-hub.ko /lib/modules/$(uname -r)/extras
# exit if something goes wrong
set -e
# load the kernel
sudo modprobe v-hub
# copy and start the overlay
sudo cp vhub-overlay.dtbo /boot/overlays
sudo dtoverlay vhub-overlay.dtbo
# print the last 5 kernel prints
dmesg  | tail -n 5
# list the loaded overlays
# you can look: cd /sys/kernel/config/device-tree/overlays/
sudo dtoverlay -l
# list the gpio, for help raspi-gpio help
raspi-gpio get 5,6

