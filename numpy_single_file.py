import numpy as np
import time
import open3d as o3d

from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("folder")
parser.add_argument("downsample")
parser.add_argument("fraction")
parser.add_argument("section")


args = parser.parse_args()

pth = Path(args.folder)
files = list(pth.rglob('*npy'))
files.sort()
print("single file prescript args processed")

def process():
  ## try reading in all the data
  start = time.perf_counter()
  arr = np.zeros((1,4))
  print("total files",len(files))
  print("fraction is",args.fraction)
  chunk_size = len(files)//int(args.fraction)
  print("total of ",chunk_size," files to aggregate in chunk")
  section_index = chunk_size*int(args.section)
  print("section index is",section_index)
  for i,f in enumerate(files[section_index:section_index+chunk_size]):
    if "full_model" in str(f) or "test" in str(f):
      if  f"dwn_{args.downsample}" in str(f) and f"section_{args.section}" in str(f):
        print("already processed to full_model format",pth)
        return
      continue
    print(i)
    new_map = np.load(f)
    arr = np.vstack([arr,new_map])

  print(arr.shape)

  #uniq = np.unique(arr,axis =0)
  #print(uniq.shape)

  #test the o3d downsample
  
  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(arr[:,:3])
  print("added pcd points")
  print("downsampling")
  downpcd = pcd.voxel_down_sample(voxel_size=float(args.downsample))
  print("downsampled")

  # save the file
  print("saving")
  np.save(f"{pth}/{pth.name}_test_model_o3d_dwn_{args.downsample}_section_{int(args.section):02}.npy",downpcd.points)
  print("done")
  end = time.perf_counter()
  print(f"seconds elapsed {end-start}")

if __name__ =="__main__":
  print("starting the numpy to full_model conversion on",args.folder,"with ",args.downsample," reduction")
  process()
