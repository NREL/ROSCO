'''
----------- Example_10 --------------
Tune a controller for distributed aerodynamic control
-------------------------------------

In this example:
- Read .yaml input file
- Load an openfast turbine model
- Read text file with rotor performance properties
- Load blade information
- Tune controller with flap actuator

Note: You will need a turbine model with DAC capabilites in order to run this. 
    The curious user can contact Nikhar Abbas (nikhar.abbas@nrel.gov) for available
    models, if they do not have any themselves. 
'''

# Python Modules
import os
# ROSCO Modules
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox.inputs.validation import load_rosco_yaml


this_dir =  os.path.dirname(os.path.abspath(__file__))

# Load yaml file
parameter_filename = os.path.join(os.path.dirname(this_dir),'Tune_Cases/BAR.yaml')
inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Load turbine data from openfast model
turbine = ROSCO_turbine.Turbine(turbine_params)
# turbine.load_from_fast(path_params['FAST_InputFile'],path_params['FAST_directory'],dev_branch=True,rot_source='txt',txt_filename=path_params['rotor_performance_filename'])
turbine.load_from_fast(path_params['FAST_InputFile'], \
    os.path.join(this_dir,path_params['FAST_directory']),dev_branch=True)

# Tune controller
controller = ROSCO_controller.Controller(controller_params)
controller.tune_controller(turbine)

print('Flap PI gains:')
print('Kp_flap = {}'.format(controller.Kp_flap[-1]))
print('Ki_flap = {}'.format(controller.Ki_flap[-1]))
