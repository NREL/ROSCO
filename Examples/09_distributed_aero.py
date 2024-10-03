"""
09_distributed_aero
-------------------
Tune a controller for distributed aerodynamic control
In this example:

* Read .yaml input file
* Load an openfast turbine model
* Read text file with rotor performance properties
* Load blade information
* Tune controller with flap actuator

Note

* You will need a turbine model with DAC capabilites in order to run this. The curious user can contact Nikhar Abbas (nikhar.abbas@nrel.gov) for available models, if they do not have any themselves. 
"""

# Python Modules
import os
# ROSCO Modules
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox.inputs.validation import load_rosco_yaml

def main():
    this_dir =  os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')

    # Load yaml file
    parameter_filename = os.path.join(tune_dir,'BAR.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Load turbine data from openfast model
    turbine = ROSCO_turbine.Turbine(turbine_params)
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory'])
        )

    # Tune controller
    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    print('Flap PI gains:')
    print('Kp_flap = {}'.format(controller.Kp_flap[-1]))
    print('Ki_flap = {}'.format(controller.Ki_flap[-1]))
 
if __name__ == "__main__":
    main()