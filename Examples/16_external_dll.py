"""
16_external_dll
---------------
Run openfast with ROSCO and external control interface
IEA-15MW will call NREL-5MW controller and read control inputs
"""

import os, platform
from rosco import discon_lib_path as lib_name
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
import shutil


def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)
    
    # Make copy of libdiscon
    ext = lib_name.split('.')[-1]
    copy_lib = os.path.join(os.path.split(lib_name)[0],f"libdiscon_copy.{ext}")
    shutil.copyfile(lib_name, copy_lib)


    # Ensure external control paths are okay
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW_ExtInterface.yaml')
    run_dir = os.path.join(example_out_dir,'16_ExtInterface')
    os.makedirs(run_dir,exist_ok=True)

    # Set DLL file and DISCON input dynamically (hard-coded in yaml)
    controller_params = {}
    controller_params['DISCON'] = {}
    controller_params['DISCON']['DLL_FileName'] =  copy_lib
    controller_params['DISCON']['DLL_InFile'] =    os.path.join(this_dir,'Test_Cases/NREL-5MW/DISCON.IN')
    controller_params['DISCON']['DLL_ProcName'] =  'DISCON'

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step
    r.wind_case_opts    = {
        'U_start': [10],
        'U_end': [15],
        'wind_dir': run_dir
        }
    r.controller_params = controller_params
    r.rosco_dir     = rosco_dir
    r.save_dir      = run_dir

    r.run_FAST()


if __name__=="__main__":
    main()
