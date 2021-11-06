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
sudo cp vhub-overlay.dtbo /boot/overlays
sudo dtoverlay vhub-overlay.dtbo
# start libcomposite
sudo modprobe libcomposite
# print the last 5 kernel prints
dmesg  | tail -n 5
# list the loaded overlays
sudo dtoverlay -l
