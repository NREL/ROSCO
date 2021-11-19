'''
----------- Example_11 --------------
Load a turbine, tune a controller, export linear model
-------------------------------------

In this example:
  - Load a turbine from OpenFAST
  - Tune a controller
  - Use tuning parameters to export linear model

'''
# Python Modules
import os
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.inputs.validation import load_rosco_yaml


import numpy as np

# Load yaml file 
parameter_filename = os.path.join( os.path.dirname( os.path.dirname( os.path.realpath(__file__) )), 
                                 'Tune_Cases', 'IEA15MW.yaml')
inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Linear file output
this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir,'examples_out')
if not os.path.isdir(example_out_dir):
  os.makedirs(example_out_dir)

linmod_filename     = os.path.join(example_out_dir,'11_IEA15MW_LinMod.dat')

# Instantiate turbine, controller, and file processing classes
turbine         = ROSCO_turbine.Turbine(turbine_params)
controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
tune_dir =  os.path.join(this_dir,'../Tune_Cases')
turbine.load_from_fast(
  path_params['FAST_InputFile'],
  os.path.join(this_dir,path_params['FAST_directory']),
  dev_branch=True,
  rot_source='txt',
  txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
  )

# Tune controller 
controller.tune_controller(turbine)

# Write Linear model parameters to text file for matlab processing
# Add to ROSCO_utilities.FileProcessing() when finished

print('Writing linear models to text file: ' + linmod_filename)

# extend gain schedule

pc_br = np.zeros(len(controller.v_below_rated))
pc_Kp = np.concatenate((pc_br,controller.pc_gain_schedule.Kp))
pc_Ki = np.concatenate((pc_br,controller.pc_gain_schedule.Ki))

vs_ar = np.zeros(len(controller.pc_gain_schedule.Kp))
vs_Kp = np.concatenate((controller.vs_gain_schedule.Kp,vs_ar))
vs_Ki = np.concatenate((controller.vs_gain_schedule.Ki,vs_ar))

with open(linmod_filename,'w') as f:
    f.write('{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\t{:12}\n'.\
        format('WindSpeed','A_om','b_theta','b_tau','b_wind','pc_Kp','pc_Ki','vs_Kp','vs_Ki','Pi_omega','Pi_theta','Pi_wind'))

    for v,A,B_beta,B_tau,B_wind,pc_Kp,pc_Ki,vs_Kp,vs_Ki,Pi_omega,Pi_beta,Pi_wind in zip(controller.v,controller.A,controller.B_beta,controller.B_tau, controller.B_wind, \
        pc_Kp, pc_Ki, vs_Kp, vs_Ki, \
        controller.Pi_omega, controller.Pi_beta, controller.Pi_wind):
        f.write('{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\t{:12.4e}\n'\
            .format(v,A,B_beta,B_tau,B_wind,pc_Kp,pc_Ki,vs_Kp,vs_Ki,Pi_omega,Pi_beta,Pi_wind))

print('Tower Height = {} m'.format(turbine.hubHt))
print('Platform Freq. = {} rad/s'.format(controller.ptfm_freq))

