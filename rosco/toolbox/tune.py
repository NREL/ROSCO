# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import os
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.utilities import read_DISCON, write_DISCON, DISCON_dict


def yaml_to_objs(tuning_yaml):
    # Load yaml and return controller, turbine objects
    # Basically, tune a whole controller!

    # Load yaml file 
    inps = load_rosco_yaml(tuning_yaml)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    yaml_dir = os.path.dirname(tuning_yaml)  # files relative to tuning yaml
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(yaml_dir,path_params['FAST_directory']),
        rot_source='txt',
        txt_filename=os.path.join(yaml_dir,path_params['rotor_performance_filename'])
        )

    # Tune controller 
    controller.tune_controller(turbine)

    return controller, turbine, path_params

def update_discon_version(file,tuning_yaml,new_discon_filename):
    '''''''''
    This function will take a DISCON input from a previous version of ROSCO and conver it to this version
    It will not be 100%, requires a check, but still saves time

    '''

    # Use new defaults for these inputs for various reasons
    exclude_list = [
        'Y_ErrThresh',      # translated from float to list of floats in v2.6
        'WE_CP',
        ]

    # Read original DISCON
    original_vt = read_DISCON(file)

    # Tune dummy controller to get objects
    controller, turbine, _ = yaml_to_objs(tuning_yaml)

    # Load default inputs
    discon_defaults = DISCON_dict(turbine,controller)
    
    # If no value in old discon, set default to start.  Note: a simple update doesn't handle type changes
    new_discon = {}
    for param in discon_defaults:
        # If the value is in the original DISCON and not excluded, use the original
        if (param in original_vt) and (param not in exclude_list):
            new_discon[param] = original_vt[param]
        
        # Otherwise, use the new default
        else:
            new_discon[param] = discon_defaults[param]

    # Now handle edge cases

    # Torque control: broken into 2 input modes after 2.9.0
    # For newer versions, where VS_ConstPower is not an option, don't do this
    if 'VS_ConstPower' not in original_vt:
        if original_vt['VS_ControlMode'] == 0:
            new_discon['VS_ControlMode'] = 1
            new_discon['VS_ConstPower'] = 0
        elif original_vt['VS_ControlMode'] == 1:
            new_discon['VS_ControlMode'] = 1
            new_discon['VS_ConstPower'] = 1
        elif original_vt['VS_ControlMode'] == 2:
            new_discon['VS_ControlMode'] = 2
            new_discon['VS_ConstPower'] = 0
        elif original_vt['VS_ControlMode'] == 3:
            new_discon['VS_ControlMode'] = 2
            new_discon['VS_ConstPower'] = 1

    # Notch filters
    if 'F_NumNotchFilts' not in original_vt:
        if original_vt['F_NotchType'] == 0:
            new_discon['F_NumNotchFilts'] = 0
        else:
            new_discon['F_NumNotchFilts'] = 1
            new_discon['F_NotchFreqs'] = [original_vt['F_NotchCornerFreq']]
            new_discon['F_NotchBetaNum'] = [original_vt['F_NotchBetaNumDen'][0]]
            new_discon['F_NotchBetaDen'] = [original_vt['F_NotchBetaNumDen'][1]]

        if original_vt['F_NotchType'] == 1 or original_vt['F_NotchType'] == 3:
            new_discon['F_GenSpdNotch_N'] = 1
            new_discon['F_GenSpdNotch_Ind'] = [1]

        if original_vt['F_NotchType'] == 2 or original_vt['F_NotchType'] == 3:
            new_discon['F_TwrTopNotch_N'] = 1
            new_discon['F_TwrTopNotch_Ind'] = [1]
    

    # Make the DISCON
    write_DISCON(
        turbine,
        controller,
        new_discon_filename,
        rosco_vt = new_discon,
    )

