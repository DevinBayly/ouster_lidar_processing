#!/bin/bash
source sourcing_melodic.sh
mkdir -p temp/logs/$4
export ROS_HOME=$(pwd)/temp/logs/$4
# work out way to check for nodisp existence and skip this chunk
python3 aloam_bag.py "$5"
# set environment var so we process to pcd
export PROC_PCD=yes
#python3 pcap_to_pcd.py "$1" "$2" $3
