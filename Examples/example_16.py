'''
----------- Example_16 --------------
Run openfast with ROSCO and external control interface
-------------------------------------

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
import shutil


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir         = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))


def main():
    
    # Make copy of libdiscon
    ext = lib_name.split('.')[-1]
    copy_lib = os.path.join(os.path.split(lib_name)[0],f"libdiscon_copy.{ext}")
    shutil.copyfile(lib_name, copy_lib)


    # Ensure external control paths are okay
    print('here')
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/IEA15MW_ExtInterface.yaml')
    run_dir = os.path.join(example_out_dir,'16_ExtInterface')
    os.makedirs(run_dir,exist_ok=True)

    # RAAW FAD set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step
    r.wind_case_opts    = {
        'U_start': [10],
        'U_end': [15],
        'wind_dir': run_dir
        }
    r.save_dir      = run_dir

    r.run_FAST()


if __name__=="__main__":
    main()