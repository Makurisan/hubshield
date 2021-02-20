# exit if something goes wrong
set -e

git rev-parse --show-toplevel
# ssh pi@pi3 "cd ~/vusb-build; source ./notify.sh"
# kernel
scp ./build/v-hub.ko pi:/home/ubuwin/vusb-build
# device tree
scp ./build/vhub-overlay.dtbo pi:/home/ubuwin/vusb-build

# delete file which we dont need
cd build
rm ./.* -rf
rm *.o -rf
cd -

