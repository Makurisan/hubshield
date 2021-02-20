# ssh pi@pi3 "cd ~/vusb-build; source ./notify.sh"
ssh pi@pi3 "clear"
# kernel
scp ./build/v-hub.ko pi@pi3:/home/pi/vusb-build
# device tree
scp ./build/hubshield-vhub.dtbo pi@pi3:/home/pi/vusb-build

