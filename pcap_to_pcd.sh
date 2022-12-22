#!/bin/bash
source sourcing.bash
mkdir -p temp/logs/$4
export ROS_HOME=$(pwd)/temp/logs/$4
echo $1 $2 $3
python3 pcap_to_pcd.py "$1" "$2" $3
