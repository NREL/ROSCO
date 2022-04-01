'''
----------- Example_14 --------------
Load a turbine, tune a controller with open loop control commands
-------------------------------------

In this example:
  - Load a turbine from OpenFAST
  - Tune a controller
  - Write open loop inputs
  - Run simple simulation with open loop control

'''
# Python Modules
import yaml, os, platform
import numpy as np
import matplotlib.pyplot as plt

# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import utilities as ROSCO_utilities
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
from ROSCO_toolbox.ofTools.case_gen.CaseLibrary import set_channels
from ROSCO_toolbox.ofTools.case_gen.runFAST_pywrapper   import runFAST_pywrapper, runFAST_pywrapper_batch
from ROSCO_toolbox.ofTools.case_gen.CaseGen_General     import CaseGen_General



this_dir          = os.path.dirname(os.path.abspath(__file__))
rosco_dir         = os.path.dirname(this_dir)
example_out_dir   = os.path.join(this_dir,'examples_out')
example_out_dir = os.path.join(this_dir,'examples_out')
if not os.path.isdir(example_out_dir):
  os.makedirs(example_out_dir)

# Load yaml file (Open Loop Case)
parameter_filename = os.path.join(rosco_dir,'Tune_Cases/IEA15MW_OL.yaml')

inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Set up open loop input
olc = ROSCO_controller.OpenLoopControl(t_max=20)
olc.interp_timeseries(
  'blade_pitch', 
  [0,20], 
  [0,0.0873] , 
  'sigma'
  )
olc.const_timeseries(
  'generator_torque', 
  19624046*.5
  )
olc.sine_timeseries('nacelle_yaw', 0.0524, 60)

# Plot open loop timeseries
fig,ax = olc.plot_timeseries()
if False:
  plt.show()
else:
  fig.savefig(os.path.join(example_out_dir,'14_OL_Inputs.png'))

# Write open loop input, get OL indices
ol_filename = os.path.join(example_out_dir,'14_OL_Input.dat')
ol_dict = olc.write_input(ol_filename)
controller_params['open_loop'] = ol_dict


# Instantiate turbine, controller, and file processing classes
turbine         = ROSCO_turbine.Turbine(turbine_params)
controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(path_params['FAST_InputFile'], \
  os.path.join(this_dir,path_params['FAST_directory']), \
    dev_branch=True,rot_source='txt',\
      txt_filename=os.path.join(this_dir,path_params['FAST_directory'],path_params['rotor_performance_filename']))

# Tune controller 
controller.tune_controller(turbine)

# Write parameter input file
param_file = os.path.join(this_dir,'DISCON.IN')   # This must be named DISCON.IN to be seen by the compiled controller binary. 
ROSCO_utilities.write_DISCON(turbine,controller,param_file=param_file, txt_filename=path_params['rotor_performance_filename'])

### Run OpenFAST using aeroelasticse tools

# Set rosco_dll
if platform.system() == 'Windows':
    rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.dll')
elif platform.system() == 'Darwin':
    rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.dylib')
else:
    rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.so')

case_inputs = {}
case_inputs[('ServoDyn','DLL_FileName')] = {'vals': [rosco_dll], 'group': 0}

# Apply all discon variables as case inputs
discon_vt = ROSCO_utilities.DISCON_dict(
  turbine, 
controller, 
txt_filename=os.path.join(this_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
)
for discon_input in discon_vt:
    case_inputs[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

case_inputs[('Fst','TMax')] = {'vals': [20], 'group': 0}
case_inputs[('InflowWind','HWindSpeed')] = {'vals': [10], 'group': 0}
case_inputs[('ElastoDyn','HWindSpeed')] = {'vals': [5.], 'group': 0}
case_inputs[('DISCON_in','LoggingLevel')] = {'vals': [3], 'group': 0}

# Generate cases
run_dir = os.path.join(example_out_dir,'14_OL_Sim')
if not os.path.exists(run_dir):
  os.makedirs(run_dir)

case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase='OL_Example')
channels = set_channels()

# Run FAST cases
fastBatch                   = runFAST_pywrapper_batch(FAST_ver='OpenFAST',dev_branch = True)

fastBatch.FAST_directory    = os.path.realpath(os.path.join(rosco_dir,'Tune_Cases',path_params['FAST_directory']))
fastBatch.FAST_InputFile    = path_params['FAST_InputFile']        
fastBatch.channels          = channels
fastBatch.FAST_runDirectory = run_dir
fastBatch.case_list         = case_list
fastBatch.case_name_list    = case_name_list
fastBatch.debug_level       = 2
fastBatch.FAST_exe          = 'openfast'

fastBatch.run_serial()


# #  Define Plot cases 
cases = {}
cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed','NacYaw']

out_file = os.path.join(example_out_dir,'14_OL_Sim/OL_Example_0.outb')
op = output_processing.output_processing()
fastout = op.load_fast_out(out_file, tmin=0)
fig, ax = op.plot_fast_out(cases=cases,showplot=False)

if False:
  plt.show()
else:
  fig[0].savefig(os.path.join(example_out_dir,'14_OL_FAST_Out.png'))


