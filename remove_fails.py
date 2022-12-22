from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("folder")


args = parser.parse_args()

fls = Path(args.folder).rglob("*pcap")


for fl in fls:
  temp = Path("temp").iterdir()
  for tempf in temp:
    if str(fl.stem) in str(tempf):
      if tempf.stat().st_size < 5000 and not tempf.is_dir():
        print("")
        print(tempf,tempf.stat().st_size)
        print("\tprobably want to remove",tempf)
        print("")
        tempf.unlink()
  
