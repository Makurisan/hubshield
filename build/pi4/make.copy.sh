# exit if something goes wrong
set -e
# build again and copy 
cd build
make
cd -
./install.remote.sh