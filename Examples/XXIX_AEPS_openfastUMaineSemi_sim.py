"""
29_AEPS_OpenfastUMaineSemi_sim
---------------
Load a turbine, tune a controller, run OpenFAST simulation, load OpenFAST output data, define the desired output parameters, load, plot& save the parameters
In this example:

* Load a turbine from OpenFAST
* Tune a controller
* Run an OpenFAST simulation
* Load the OpenFAST output data
* Define the desired OpenFAST output parameters
* Load, plot and save the parameters

Note

* You will need to have a compiled controller in ROSCO/build/ 
"""

# Python Modules
#import yaml
import os
#import numpy as np
import matplotlib.pyplot as plt
# ROSCO toolbox modules 
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.utilities import write_DISCON, run_openfast
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco import discon_lib_path

def main():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    example_out_dir = os.path.join(this_dir,'examples_out')

    # Load yaml file 
    parameter_filename = os.path.join(tune_dir, 'IEA15MW_MultiOmega_AEPS.yaml')
    #parameter_filename = os.path.join(tune_dir, 'NREL5MW_AEPS.yaml') 
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    turbine.load_from_fast(
    path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory']),
        rot_source='txt',
        txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
        )

    # EKLEMEYİ BURAYA YAPTIM
    # Setting up the location of ROSCO library
    turbine.fast.fst_vt['ServoDyn']['DLL_FileName'] = discon_lib_path

    # Tune controller 
    controller.tune_controller(turbine)

    # Write parameter input file
    param_file = os.path.join(this_dir,'DISCON.IN')   # This must be named DISCON.IN to be seen by the compiled controller binary. 
    write_DISCON(turbine,controller,param_file=param_file, txt_filename=path_params['rotor_performance_filename'])

    # Run OpenFAST
    # --- May need to change fastcall if you use a non-standard, conda-installed command to call openfast
    # If you run the `fastcall` from the command line where you run this script, it should run OpenFAST
    fastcall = 'openfast'
    run_openfast(
    os.path.join(tune_dir,path_params['FAST_directory']),
    fastcall=fastcall, 
    fastfile=path_params['FAST_InputFile'], 
    chdir=True
    )

    # Define openfast output filenames
    filenames = ["Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT-UMaineSemi_AEPS/IEA-15-240-RWT-UMaineSemi.outb"]
    # ---- Note: Could load and plot multiple cases, textfiles, and binaries...
    #filenames = ["Test_Cases/NREL-5MW/NREL-5MW.outb"]

    filenames = [os.path.join(this_dir,file) for file in filenames]

    #  Define Plot cases 
    #  --- Comment,uncomment, create, and change these as desired...
    cases = {}
    cases['AEPS'] = ['Wind1VelX', 'BldPitch1', 'RtFldFxh', 'RotSpeed','GenPwr']
    #cases['Rotor'] = ['BldPitch1', 'GenTq', 'GenPwr']
    #cases['Platform Motion'] = ['PtfmSurge', 'PtfmSway', 'PtfmHeave', 'PtfmPitch','PtfmRoll','PtfmYaw']

    # Instantiate fast_IO
    fast_out = output_processing.output_processing()
    # Can also do:
    # fast_out = output_processing.output_processing(filenames=filenames, cases=cases)
    # fast_out.plot_fast_out()

    # Load and plot
    fastout = fast_out.load_fast_out(filenames)
    fast_out.plot_fast_out(cases=cases,showplot=False)

    plt.savefig(os.path.join(example_out_dir,'29_AEPS_IEA-15MW_Semi_Out.png'))
    #plt.savefig(os.path.join(example_out_dir,'29_AEPS_5MW_Out.png'))

if __name__ == "__main__":
    main()