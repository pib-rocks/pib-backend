#!/bin/bash

WORK_DIR=$1

cat << EOF
#!/bin/bash
source $WORK_DIR/install/setup.bash
ros2 run oak_d_lite stereo
EOF