'''
----------- Example_13 --------------
Load a turbine, tune a controller with IPC
-------------------------------------

In this example:
  - Load a turbine from OpenFAST
  - Tune a controller with IPC
  - Run simple simulation with open loop control

'''
# Python Modules
import yaml, os, platform
import numpy as np
import matplotlib.pyplot as plt

# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.utilities import DISCON_dict
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
from ROSCO_toolbox.ofTools.case_gen.CaseLibrary import set_channels
from ROSCO_toolbox.ofTools.case_gen.runFAST_pywrapper   import runFAST_pywrapper_batch
from ROSCO_toolbox.ofTools.case_gen.CaseGen_General     import CaseGen_General



this_dir          = os.path.dirname(os.path.abspath(__file__))
rosco_dir         = os.path.dirname(this_dir)
example_out_dir   = os.path.join(this_dir,'examples_out')
run_dir = os.path.join(example_out_dir, '13_ipc_sim')

# Load yaml file (Open Loop Case)
parameter_filename = os.path.join(rosco_dir,'Tune_Cases/BAR.yaml')

inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Turn flaps off and IPC on
controller_params['Flp_Mode'] = 0
controller_params['IPC_ControlMode'] = 1

# Instantiate turbine, and controller
turbine         = ROSCO_turbine.Turbine(turbine_params)
controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(path_params['FAST_InputFile'],
                       os.path.join(this_dir, path_params['FAST_directory']), dev_branch=True)
# Tune controller 
controller.tune_controller(turbine)

# Set rosco_dll
if platform.system() == 'Windows':
    rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.dll')
elif platform.system() == 'Darwin':
    rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.dylib')
else:
    rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.so')

case_inputs = {}
case_inputs[('ServoDyn','DLL_FileName')] = {'vals': [rosco_dll], 'group': 0}
case_inputs[('ServoDyn','Ptch_Cntrl')]   = {'vals': [1], 'group': 0}

# Apply all discon variables as case inputs
discon_vt = DISCON_dict(
            turbine, 
            controller, 
            txt_filename=os.path.join(this_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
            )

for discon_input in discon_vt:
    case_inputs[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

# Generate cases
if not os.path.exists(run_dir):
  os.makedirs(run_dir)

case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase='ipc_example')
channels = set_channels()
channels['BldPitch2'] = True
channels['BldPitch3'] = True

# Run FAST cases
fastBatch                   = runFAST_pywrapper_batch(FAST_ver='OpenFAST',dev_branch = True)

fastBatch.FAST_directory    = os.path.realpath(os.path.join(rosco_dir,'Tune_Cases',path_params['FAST_directory']))
fastBatch.FAST_InputFile    = path_params['FAST_InputFile']        
fastBatch.channels          = channels
fastBatch.FAST_runDirectory = run_dir
fastBatch.case_list         = case_list
fastBatch.case_name_list    = case_name_list
fastBatch.debug_level       = 2
fastBatch.FAST_exe = 'openfast'

fastBatch.run_serial()


# #  Define Plot cases 
cases = {}
cases['Baseline'] = ['Wind1VelX', ('BldPitch1', 'BldPitch2', 'BldPitch3'), 'RootMyc1', 'RotSpeed']

out_file = os.path.join(example_out_dir,'13_ipc_sim/ipc_example_0.outb')
op = output_processing.output_processing()
fastout = op.load_fast_out(out_file, tmin=0)
fig, ax = op.plot_fast_out(cases=cases,showplot=False)
if False:
  plt.show()
else:
  fig[0].savefig(os.path.join(example_out_dir,'13_ipc_FAST_Out.png'))



