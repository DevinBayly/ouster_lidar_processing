#!/bin/bash
#SBATCH --output=R-%x.%j.%N.out
#SBATCH --job-name=process_sama_lidar
#SBATCH --ntasks=4
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=4
#SBATCH --nodes=4             
#SBATCH --time=4:00:00   
#SBATCH --partition=standard
#SBATCH --account=visteam   
 
# SLURM Inherits your environment. cd $SLURM_SUBMIT_DIR not needed
srun bash process_all_srun.sh $1
