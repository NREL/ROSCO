#!/bin/bash
#SBATCH --account=ssc
#SBATCH --time=1:00:00
#SBATCH --job-name=rosco_test
#SBATCH --nodes=1             # This should be nC/36 (36 cores on eagle)
#SBATCH --ntasks-per-node=36
#SBATCH --mail-user dzalkind@nrel.gov
#SBATCH --mail-type BEGIN,END,FAIL
#SBATCH --output=output.%j.out
#SBATCH --partition=debug

nDV=1 # Number of design variables (x2 for central difference)
nOF=60  # Number of openfast runs per finite-difference evaluation
nC=$((nDV + nDV * nOF)) # Number of cores needed. Make sure to request an appropriate number of nodes = N / 36
## nC=72

# module load conda
# conda activate rt-env
source activate /home/dzalkind/.conda-envs/rosco-env
# which python

# module purge
# module load conda
# module load comp-intel intel-mpi mkl


mpirun -n 36 python run_Testing.py
#  python weis_driver.py
