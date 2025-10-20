"""
35_BEPS_Simulation
------------------
Basic Envelope Protection System (BEPS) for Wind Turbines
This example demonstrates a basic Envelope Protection System for wind turbines. Compared to the AEPS system in Example 34, this system does not include any Neural Network or Observer and can be considered a simplified version.
As in AEPS, an eps_percent parameter is defined in this algorithm, and the predefined thrust limit is calculated based on the turbine’s maximum thrust using this parameter. The offset between the predefined thrust limit (PreDf_Thrst) and the turbine thrust (Thrst) is used to calculate an avoidance signal through a limit avoidance design parameter, e_dp. This avoidance signal is then applied to modify the blade pitch controller output, i.e., the blade pitch reference, collectively increasing the blade pitch angles when the turbine approaches the limit. This avoidance action allows the turbine to operate safely within the defined thrust limit.
The BEPS system generates an avoidance signal when the turbine thrust approaches within five percent of the predefined limit, accounting for the delay in blade pitch response after the avoidance signal is applied. In this system, the user only needs to tune the limit avoidance design parameter, e_dp. A value around 0.5 or less might be choosen for this parameter. A high value might cause oscillation in simulations.

"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#from rosco.toolbox.ofTools.fast_io import output_processing
from openfast_io.FAST_reader import InputReader_OpenFAST
from rosco.toolbox.inputs.validation import load_rosco_yaml

import numpy as np

def main():
    rpm2RadSec = 2.0*(np.pi)/60.0

    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW_BEPS.yaml')
    run_dir = os.path.join(example_out_dir,'35_BEPS_Simulation')
    os.makedirs(run_dir,exist_ok=True)

    # Read initial input file
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']

    reader = InputReader_OpenFAST()
    reader.FAST_InputFile = path_params['FAST_InputFile']
    reader.FAST_directory = os.path.join(this_dir,'Tune_Cases',path_params['FAST_directory'])
    reader.execute()
  
    # Set DISCON input dynamically through yaml/dict
    controller_params = {}     
    controller_params['PS_Mode'] = 0
    controller_params['PA_Mode'] = 2  
    controller_params['VS_ControlMode'] = 2.
    
    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    
    # Wind case: turbulence
    r.wind_case_fcn = cl.turb_bts  
    r.wind_case_opts    = {
         'TMax': 720,  # from 10 to 15 m/s
         #'wind_filenames': ['/Users/dzalkind/Downloads/heavy_test_1ETM_U6.000000_Seed603.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/TurbSim9.bts'],
         'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U11.000000_Seed1501552846.0_DLC1.1_11ms_720s.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U18.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U20.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U22.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U24.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U26.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U28.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U30.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U11.000000_Seed488200390.0-DLL1.1_11ms_720sn_Seed2.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U11.000000_Seed1501552846.0-DLC1.1_11ms_2500sn_Seed1yeni.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U8.000000_Seed1501552846.0-DLC 1.1-720sn.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U15.000000_Seed1501552846.0-DLC 1.1-720sn.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U9.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U14.000000_Seed1501552846.0.bts'],
         }

    # Run with and without AEPS algorithm
    r.control_sweep_fcn = cl.sweep_yaml_input
    r.control_sweep_opts = {
            'control_param':'ASO_Mode',
            'param_values': [0,2] #Run with ASO mode equal to 0 and 2, respectively.
        }
    
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.case_inputs = {}
    r.rosco_dll = "C:/Users/musah/ROSCO/rosco/controller/build/libdiscon.dll"
    r.n_cores = 1
    r.run_FAST()


if __name__=="__main__":
    main()
