
import multiprocessing as mp
import argparse
from pathlib import Path
import os
import subprocess as sp

def process(inputs):
  fname = inputs[1]
  index = inputs[0]
  
  pth = fname.parent
  just_name = fname.stem
  sp.run(f"python3 pcap_to_pcd.py '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 128 {index}",shell=True)
  #print(f"python3 pcap_to_pcd.py '{pth}/{just_name}.pcap' '{pth}/{just_name}.json' 128")


parser = argparse.ArgumentParser()
parser.add_argument("folder")

args = parser.parse_args()

pcaps = [(i,pth) for i,pth in enumerate(Path(args.folder).rglob("*pcap"))]

for val in pcaps[:2]:
  process(val)
