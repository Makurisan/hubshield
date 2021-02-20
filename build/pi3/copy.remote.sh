 git rev-parse --show-toplevel
# ssh pi@pi3 "cd ~/vusb-build; source ./notify.sh"
ssh pi@pi3 "clear"
# kernel
scp ./build/v-hub.ko pi@pi3:/home/pi/vusb-build
# device tree
scp ./build/vhub-overlay.dtbo pi@pi3:/home/pi/vusb-build

# delete file which we dont need
cd build
find ./build/.* | xargs rm -fr
find ./build/*.o | xargs rm -fr
cd -

