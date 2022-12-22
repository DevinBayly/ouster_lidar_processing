import zipfile
## goal of script is to be run after pcds are created and then convert each to a npy format also
## also attempts to zip this up in the end
import numpy as np
import os
from pathlib import Path
import argparse
import multiprocessing as mp

print("starting the pcd_to_numpy.py script")


parser = argparse.ArgumentParser()
parser.add_argument("folder")

args = parser.parse_args()

parent =Path(args.folder)
print("parent is",parent)
pcds = list(parent.rglob("*pcd"))

def process(zipname):
  cutoff_string = b"DATA binary\n"
  buf = open(f"{zipname}","rb").read()
  pos = buf.find(cutoff_string) + len(cutoff_string)
  header = buf[:pos].decode()
  point_count = int([l.split(" ") for l in header.split("\n") if "POINTS" in l][0][1])
  print("points in pcd",point_count)
  binary_data = buf[pos:]
  ## calculate the number of 32 byte points in the file
  ## in our data the fields amount to 32 bytes worth of data per point
  num_bytes = 32*point_count
  byts = binary_data[:num_bytes]
  arr = np.frombuffer(byts,dtype="uint8")
  arr.dtype = np.dtype([("x","float32"),("y","float32"),("z","float32"),("skip","float32"),("intensity","float32"),("skip1","float32"),("skip2","float32"),("skip3","float32")])
  just_important = np.column_stack([arr["x"],arr["y"],arr["z"],arr["intensity"]])
  np.save(f"{parent}/{zipname.stem}.npy",just_important)

num_cpus = os.environ["SLURM_CPUS_ON_NODE"]
npys = list(parent.rglob("*npy"))

print(len(npys) , len(pcds))
zipname = f"{parent.name}".replace("_map_nodisp-pcds","")
if len(npys) < len(pcds):
  print("no npys found")
  with mp.Pool(int(num_cpus)) as p:
    p.map(process,pcds)
  # zip up the results
new_zip = parent/f"{zipname}_npys.zip"
if new_zip.exists():
  sz = os.stat(new_zip).st_size
npys = list(parent.rglob("*npy"))
if not (parent/f"{zipname}_npys.zip").exists() or sz < 50: # most failed zips are 22bytes
  print(f"no zip file {zipname}_npys.zip found for, or size indicates failure")
  print("now trying to zip")
  with zipfile.ZipFile(f"{parent}/{zipname}_npys.zip","w") as zf:
    for npy in npys:
      if not "full_model" in str(npy):
        print(npy)
        zf.write(npy)
  #process(pcds[0])
print("ending script")
