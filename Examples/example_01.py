'''
----------- Example_01 --------------
Load and save a turbine model
-------------------------------------
In this example:
- Read .yaml input file
- Load an openfast turbine model
- Read text file with rotor performance properties
- Print some basic turbine properties
- Save the turbine as a picklle

Note: Uses the NREL 5MW included in the Test Cases and is a part of the OpenFAST distribution
'''

# Python Modules
import os
# ROSCO Modules
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.inputs.validation import load_rosco_yaml


# Load yaml file
this_dir = os.path.dirname(os.path.abspath(__file__))
tune_dir =  os.path.join(this_dir,'../Tune_Cases')
parameter_filename = os.path.join(tune_dir,'NREL5MW.yaml')
inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']

# Load turbine data from openfast model
turbine = ROSCO_turbine.Turbine(turbine_params)

turbine.load_from_fast(
    path_params['FAST_InputFile'],
    os.path.join(tune_dir,path_params['FAST_directory']),
    dev_branch=True,
    rot_source='txt',txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
    )

# Print some basic turbine info
print(turbine)

# Save the turbine model
example_out_dir = os.path.join(this_dir,'examples_out')
if not os.path.isdir(example_out_dir):
  os.makedirs(example_out_dir)

turbine.save(os.path.join(example_out_dir,'01_NREL5MW_saved.p'))