'''
file: pcap_to_pcd.py
author: Ben Kruse
version: 1.0
date: 6/2/2022
'''

import sys
import os
import subprocess
import time
from pathlib import Path

def check_temp_contents(fname):
    print("comparing",fname,"against")
    for existing in os.listdir("temp"):
      print("\t",existing)
      if Path(fname).stem in existing:
        print("found a match on ",existing)
        return True
    return False

def pcap_to_bag(pcap_file_name: str, number_of_sensors: int) -> None:
    ''' Generates a bag file containing the pcap data to use in other steps

    Inputs:
    str pcap_file_name: the path of the pcap_file
    int number of sensors: the sensor count of the lidar data

    Output:
    returns: str: the path of the new bag
    Generates a bag file with the same name as the pcap in the ./temp directory
    '''
    print("running pcap to bag")
    base_pcap_name = os.path.basename(pcap_file_name)  # Gets the base name of the pcap (without the path)
    output_bag_file = os.path.abspath(f"./temp/{base_pcap_name[:-4]}bag")
    if not check_temp_contents(output_bag_file):
      command = f"./pcaptorosbag_ws/devel/lib/pcap_to_bag/pcap_to_bag '{os.path.abspath(pcap_file_name)}' '{output_bag_file}' {number_of_sensors}"
      print(command)  # To show the command
      subprocess.run(command, shell=True)  # Runs the command in the shell
    else:
      print("not re-running")
    return output_bag_file


def pcap_to_pointcloud(pcap_bag_name, json_file_name):
    ''' Retrieves the pointcloud data in the form of a bag from a pcap bag
    Inputs:
    str pcap_bag_name: the path to the pcap_bag
    str json_file_name: the path to the json_file

    Outputs:
    returns: str: the path of the new bag
    Generates a bag file named {name_of_base_bag}-pc.bag in the temp directory
    returns a string bag name with the temp/ path at the front
    '''
    output_pc_name = f"{pcap_bag_name[:-4]}-pc"
    if not check_temp_contents(output_pc_name):
      print("Starting roscore and ouster ros")
      
      roscore_proc = subprocess.Popen(f"roslaunch ouster_ros ouster.launch replay:=true metadata:='{os.path.abspath(json_file_name)}'", shell=True)
      time.sleep(10)
      
      print("Starting recording")
      recording_proc = subprocess.Popen(f"rosbag record /os_cloud_node/points /os_cloud_node/imu -O '{pcap_bag_name[:-4]}-pc' __name:=recording_proc", shell=True)
      time.sleep(10)

      print("Starting playback")
      subprocess.run(f"rosbag play '{pcap_bag_name}'", shell=True)
      time.sleep(2)

      subprocess.run("rosnode kill /recording_proc", shell=True)
      roscore_proc.terminate()
    return output_pc_name+".bag"


def bag_to_pcd_unregistered(pc_bag_name):
    ''' Creates pcd files from a bag of pointcloud2 points
    Inputs:
    str pc_bag_name: The path to the pointclound2 bag

    Outputs:
    returns None
    Creates a directory in temp called {pcap_file_name}-pcds containing all the pcd's with the timestamp as their name
    '''
    new_folder_name = os.path.basename(pc_bag_name)[:-4] + "-pcds"
    print("making new folder for pcds", new_folder_name)
    ## check if anything exists 
    
    folder = Path(f"./temp/{new_folder_name}")
    number_of_files = 0
    if folder.exists():
      number_of_files = len(list(folder.iterdir()))
    if not folder.exists() or number_of_files == 0:
      print("no matches for ",new_folder_name, " making pcds")
      folder.mkdir(exist_ok=True)
      roscore = subprocess.Popen("roscore",shell=True)
      print(f"rosrun pcl_ros bag_to_pcd '{os.path.abspath(pc_bag_name)}' /os_cloud_node/points './temp/{new_folder_name}'")
      subprocess.run(f"rosrun pcl_ros bag_to_pcd '{os.path.abspath(pc_bag_name)}' /os_cloud_node/points './temp/{new_folder_name}'", shell=True)
      roscore.terminate()
      print("finished doign the pcds")
    

def bag_to_pcd(pc_bag_name):
    ''' Creates pcd files from a bag of pointcloud2 points, this is used on the aloam results
    Inputs:
    str pc_bag_name: The path to the pointclound2 bag

    Outputs:
    returns None
    Creates a directory in temp called {pcap_file_name}-pcds containing all the pcd's with the timestamp as their name
    '''
    #haifa5 OS-1-128-122133000772-2048x10_map_nodisp
    new_folder_name = os.path.basename(pc_bag_name)[:-4] + "-pcds"
    print("making new folder for pcds", new_folder_name)
    if not check_temp_contents(new_folder_name):
      print("no matches for ",new_folder_name, " making pcds")
      subprocess.run(f"mkdir -p './temp/{new_folder_name}'", shell=True)
      roscore = subprocess.Popen("roscore",shell=True)
      subprocess.run(f"rosrun pcl_ros bag_to_pcd '{os.path.abspath(pc_bag_name)}' /velodyne_cloud_registered './temp/{new_folder_name}'", shell=True)
      roscore.terminate()
    


def main():
    ''' Command line arguments:
        pcap file name
        json file name
        number of sensors (16, 32, 64, 128)
            This is usually part of the pcap name
    '''
    subprocess.run("pkill roscore",shell=True)
    subprocess.Popen("roscore",shell=True)
    time.sleep(2)

    if len(sys.argv) < 4:
            print("Must have pcap name, json file, and number of sensors as arguments")
            return 1
    pcap_file_name = sys.argv[1]
    json_file_name = sys.argv[2]
    number_of_sensors = sys.argv[3]
    

    # Converting the pcap file to a bag file
    pcap_bag_name = pcap_to_bag(pcap_file_name, number_of_sensors)
##
##    # Converting the pcap bag to a pointcloud2 bag
    pc_bag_name = pcap_to_pointcloud(pcap_bag_name, json_file_name)
##    subprocess.run("rostopic list", shell=True)
    bag_to_pcd_unregistered(pc_bag_name)
    if os.getenv("PROC_PCD"):
      print("got command to do PROC_PCD")
      pcap_path = Path(pcap_file_name)
    #haifa5 OS-1-128-122133000772-2048x10_map_nodisp
      pc_file_name = Path(f"./temp/{pcap_path.stem}_map_nodisp.bag")
      print(pc_file_name,"pcfilename is")
      bag_to_pcd(pc_file_name)

      print("It worked!")
    subprocess.run("pkill roscore",shell=True)
    time.sleep(2)



if __name__ == "__main__":
    main()

