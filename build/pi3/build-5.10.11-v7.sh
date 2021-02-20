# delete all file 
 rm -rf ./build/*; rm -rf ./build/.*
 # copy the source
cp -r ../../source/* ./build
# copy kernel specific makefile to build directory 
cp Makefile.5.10.11-v7 ./build/Makefile
# copy dts
cp ../vhub-pi-overlay.dts ./build/vhub-pi-overlay.dts
# build dts
cd build
dtc -@ -I dts -O dtb -o vhub-overlay.dtbo vhub-pi-overlay.dts
# make the kernel module
make
cd -