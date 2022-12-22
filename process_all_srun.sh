#!/bin/bash
#hostname
#nproc
folder_to_process=$1

python3 remove_fails.py $folder_to_process
python3 process_all.py $folder_to_process $SLURM_NODEID
#cd pcd_processing
#python3 paralleler.py
