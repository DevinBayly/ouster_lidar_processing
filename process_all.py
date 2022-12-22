
import multiprocessing as mp
import argparse
from pathlib import Path
import os
import subprocess as sp

def process(fname):
  pth = fname.parent
  just_name = fname.stem
  stem_underscores = just_name.replace(" ","_")
  #print(f"singularity exec ros_latest.sif /bin/bash pcap_to_pcd.sh '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 128 {stem_underscores}")
# use this line when the scanner is older
  #sp.run(f"singularity exec ros_latest.sif /bin/bash pcap_to_pcd.sh '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 16 {stem_underscores}",shell=True)
  sp.run(f"singularity exec ros_latest.sif /bin/bash pcap_to_pcd.sh '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 128 {stem_underscores}",shell=True)
  #switch containers
  # this will do the aloam and bag to pcd steps
  print("starting to run the pcd conversion step")
  ##sp.run(f"singularity exec ros_lidar_latest.sif /bin/bash odometry_and_mapping.sh '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 16 '{stem_underscores}' '{just_name}'",shell =True)
  sp.run(f"singularity exec ros_lidar_latest.sif /bin/bash odometry_and_mapping.sh '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 128 '{stem_underscores}' '{just_name}'",shell =True)
  ## finish by converting the pcds to numpy format
  #sp.run(f'singularity exec ouster_sama_latest.sif python3 pcd_to_numpy.py "temp/{just_name}_map_nodisp-pcds"',shell=True)
  print("now making the full_model")
  #sp.run(f'singularity exec ouster_sama_latest.sif python3 numpy_single_file.py "temp/{just_name}_map_nodisp-pcds" .5',shell=True)

parser = argparse.ArgumentParser()
parser.add_argument("folder")
parser.add_argument("nid")

args = parser.parse_args()

# remove the [0] index to process more files
pcaps = [pth for pth in Path(args.folder).rglob("*pcap")]

finished = Path("finished")
finished.mkdir(exist_ok=True)
for i,pth in enumerate(pcaps):
  if i%4 == int(args.nid):
    print("process all runnin,",pth)
    process(pth)
    # remove the unnec files to save on storage
    #sp.run(f"mv 'temp/{pth.stem}-pc-pcds' finished",shell=True)
    #sp.run(f"rm 'temp/{pth.stem}.bag' 'temp/{pth.stem}-pc.bag'",shell=True)

