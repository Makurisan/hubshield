# execute 'copy-build.sh', if v-hub.ko has been changed
# to install: 
#  git clone https://github.com/eradman/entr.git
find v-hub.ko | entr sh -c 'clear; ./copy-build.sh'

