import sys
import os
import subprocess
import time
import argparse
import subprocess as sp
from pathlib import Path



def check_temp_contents(fname):
  print("comparing",fname,"against")
  for existing in os.listdir("temp"):
    print("\t",existing)
    if Path(fname).stem in existing:
      return True
  return False
def run_aloam_process(fname):
  print("starting aloam process")
  if not check_temp_contents(f"{fname}_map_nodisp"):
    aloam_proc = sp.Popen("roslaunch aloam_ouster aloam_ouster.launch",shell =True)
    #record = sp.Popen(f'rosbag record -O "temp/{fname}_map_nodisp" /velodyne_cloud_registered /laser_cloud_map',shell=True)
    record = sp.Popen(f'rosbag record -O "temp/{fname}_transforms" /tf /tf_static',shell=True)
    #play the file
    sp.run(f'rosbag play "temp/{fname}-pc.bag"',shell =True)
    aloam_proc.terminate()
    record.terminate()
  else:
    print("no aloam needed for",fname)

if __name__ == "__main__":

  parser = argparse.ArgumentParser()
  parser.add_argument("basename")

  args = parser.parse_args()
  run_aloam_process(args.basename)
