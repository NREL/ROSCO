"""
07_openfast_outputs
-------------------
Demonstrate the ROSCO routines for reading OpenFAST I/O files.
Nearly all the figures in the example documentation rely on a version of these scripts, which can also be used to load ROSCO *.dbg* files, including

 # .dbg files output basic ROSCO internal variables, like the inputs and outputs to the wind speed estimator
 # .dbg2 files output all the ROSCO LocalVariables.  If the variable is an array, only the first entry is recorded.
 # .dbg3 files output the avrSWAP array at each timestep that is transfered between ROSCO and the multi-physics solver

Note: this example relies on the previous running of the OpenFAST model in 'Test_Cases/NREL-5MW/' to plot.
"""

# Python Modules
#import numpy as np
import matplotlib.pyplot as plt 
# ROSCO toolbox modules 
from rosco.toolbox.ofTools.fast_io import output_processing
import os

def main():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    example_out_dir = os.path.join(this_dir,'examples_out')
    if not os.path.isdir(example_out_dir):
        os.makedirs(example_out_dir)

    # Define openfast output filenames
    filenames = ["Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT-UMaineSemi/IEA-15-240-RWT-UMaineSemi.outb"]
    # ---- Note: Could load and plot multiple cases, textfiles, and binaries...
    # filenames = ["../Test_Cases/NREL-5MW/NREL-5MW.outb",
    #             "../Test_Cases/NREL-5MW/NREL-5MW_ex8.outb"]

    filenames = [os.path.join(this_dir,file) for file in filenames]

    #  Define Plot cases 
    #  --- Comment,uncomment, create, and change these as desired...
    cases = {}
    cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed']
    # cases['Rotor'] = ['BldPitch1', 'GenTq', 'GenPwr']
    # cases['Platform Motion'] = ['PtfmSurge', 'PtfmSway', 'PtfmHeave', 'PtfmPitch','PtfmRoll','PtfmYaw']


    # Instantiate fast_IO
    fast_out = output_processing.output_processing()
    # Can also do:
    # fast_out = output_processing.output_processing(filenames=filenames, cases=cases)
    # fast_out.plot_fast_out()

    # Load and plot
    fastout = fast_out.load_fast_out(filenames)
    fast_out.plot_fast_out(cases=cases,showplot=False)

    plt.savefig(os.path.join(example_out_dir,'07_IEA-15MW_Semi_Out.png'))

if __name__ == "__main__":
    main()
