# exit if something goes wrong
set -e
# build again and copy 
cd build
make
cd -
./copy.remote.sh