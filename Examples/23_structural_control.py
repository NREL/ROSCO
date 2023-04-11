'''
----------- 23_structural_control ------------------------
Run openfast with ROSCO and structural control
-----------------------------------------------

Set up and run simulation with pitch offsets, check outputs

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
import numpy as np
from ROSCO_toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
from ROSCO_toolbox.inputs.validation import load_rosco_yaml

'''
ROSCO currently supports user-defined hooks for structural control control actuation, if StC_Mode = 1.
The control logic can be determined in Controllers.f90 with the StructrualControl subroutine.
In the DISCON input, users must specify StC_GroupIndex relating to the control ChannelID.  
These indices can be found in the ServoDyn summary file (*SrvD.sum)

In the example below (and hard-coded in ROSCO) a step change of -4e5 N on the first structural controller 
is applied at 50 sec.

The develop branch (as of Mar 3, 2023) of OpenFAST (v3.5.0, upcoming) is required to run this example
'''


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))


def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/IEA15MW_ballast.yaml')
    run_dir = os.path.join(example_out_dir,'23_structural_control')
    os.makedirs(run_dir,exist_ok=True)

    # Read initial input file
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']

    # Change inputs programatically, read first
    reader = InputReader_OpenFAST()
    reader.FAST_InputFile = path_params['FAST_InputFile']
    reader.FAST_directory = os.path.join(rosco_dir,'Tune_Cases',path_params['FAST_directory'])
    # reader.FAST_directory = '/Users/dzalkind/Tools/ROSCO1/Test_Cases/ptfm_control_archive/IEA-15-240-RWT-UMaineSemi_ballast'
    reader.execute()

    reader.fst_vt['ServoDyn']['NumSStC'] = 3
    reader.fst_vt['ServoDyn']['SStCfiles'] = ['StC-Force-Col1.dat', 'StC-Force-Col2.dat', 'StC-Force-Col3.dat']
    # Add SStC file inputs
    for StC_file in reader.fst_vt['ServoDyn']['SStCfiles']:
        reader.fst_vt['SStC'].append(reader.read_StC(StC_file))


    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [9],
        'T_max': 100,
        }
    r.case_inputs = {}
    r.fst_vt        = reader.fst_vt
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()



if __name__=="__main__":
    main()