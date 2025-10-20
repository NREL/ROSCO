"""
34_AEPS_Simulation
------------------
Adaptive Envelope Protection System (AEPS) for Wind Turbines

This example demonstrates the Adaptive Envelope Protection System (AEPS) for a wind turbine. It presents simulation results with the AEPS system disabled (ASO = 0) and enabled (ASO = 1).
The AEPS is designed to protect turbines from excessive loads, i.e., excessive thrust force. It is an adaptive algorithm that learns the turbine’s operating conditions in real time and ensures its safe operation within a predefined thrust limit. By monitoring wind and turbine states such as wind speed, thrust force, and rotor speed, AEPS online adapts to the turbine’s operational conditions through an adaptive neural network (LPNN) and predicts the near-future thrust response.
When the predicted thrust response is about to exceed the predefined thrust limit, AEPS collectively changes the turbine blade pitch angles to ensure safe operation. This is achieved by calculating a theoretical wind speed, referred to as the envelope wind speed, and comparing it with the actual wind speed to determine whether the turbine is operating under excessive thrust or not. If the actual wind speed is lower than the envelope wind speed, the turbine operates safely; otherwise, it experiences excessive thrust force.
When the thrust is close to exceeding the predefined limit, AEPS generates an additional blade pitch reference signal through a limit avoidance design parameter based on the offset between the actual and envelope wind speeds. This additional signal is added to the blade pitch controller output, thereby collectively increasing the blade pitch angles. This avoidance action reduces thrust force and keeps the turbine within the predefined safe limit, ensuring continued safe operation of the turbine.
More information on AEPS can be found in:
Sahin, M., Yavrucuk, I., Adaptive envelope protection control of wind turbines under varying operational conditions, Energy, Volume 247, 2022, 123544, ISSN 0360-5442, https://doi.org/10.1016/j.energy.2022.123544.


Current AEPS Design and Implementation

In the current implementation, different from the above reference, thrust and wind speed estimates, and filtered rotor speed are used in AEPS rather than their actual values. In addition, since blade pitch response exhibits a lag when AEPS applies an avoidance signal, the system is activated earlier: when the actual wind speed approaches the calculated envelope wind speed. Also, a squashing function is used for the NN inputs.
Finally, the predefined thrust force is calculated based on the maximum thrust force of the turbine using an eps_percent parameter. When the user selects an eps_percent value and activates the AEPS system (ASO = 1), the algorithm determines whether the turbine will operate under excessive thrust conditions and changes the blade pitch controller output, only when necessary, thereby keeping the thrust force within the selected limit.

AEPS Tuning Procedure

The AEPS implementation is realized such that the user can tune the following three parameters:
•	Observer gain (Kc)
•	Neural network learning rate (Γ / Gamma)
•	e-modification term gain (k)
•	Effective limit avoidance parameter (e_dp)
AEPS tuning can be started with selecting the observer gain (Kc) and neural network (NN) learning rate (Γ) when AEPS is disabled (ASO = 0). The key purpose is to obtain an AEPS-predicted thrust force (Thrst_est) that is very close to the turbine thrust force (Thrst). The designer can compare these two parameters (Thrst_est and Thrst). For ease of comparison, the designer can also check the error between them (T_error = Thrst – Thrst_est), which should be close to zero.
Greater emphasis can be first placed on tuning the observer gain Kc, which controls the convergence rate of the algorithm. Values such as 2, 3, 4, 5 and so on can be tested, with the goal of minimizing T_error. Once a suitable Kc is chosen, the NN learning rate Γ can be adjusted to further reduce the error. The e-modification term gain (ke) is usually a small number such as 0.01, 0.02, etc. Nevertheless, it might take higher values such as 0.5, or 1. Small numbers such as 0.02 are recommended for ke.
The learning rate Γ determines how quickly the neural network adapts to turbine operating conditions, i.e., predicting the turbine thrust force. A higher learning rate Γ results in faster learning but high oscillations in the AEPS-predicted thrust force. Once adaptation is achieved, the AEPS-predicted thrust (Thrst_est) should closely track the turbine thrust force (Thrst). Here, it should be also noted that the adaptation/learning is satisfied for a range of these tuning values. Therefore, the designer should decide all of these values based on the satisfied error.
When the observer gain, learning rate and e-modification term gain are tuned as desired, AEPS can be activated (ASO = 1) to tune the effective limit avoidance parameter (e_dp). This parameter is highly sensitive to the turbine actuator. If the actuator model changes, e_dp may need to be re-tuned. It should be tuned such that the turbine thrust stays within the predefined thrust limit value. It should be kept in mind that choosing a high value for e_dp might cause oscillations in simulations.
Finally, if desired, after AEPS activation, the designer may fine-tune the observer gain and learning rate, again by comparing AEPS-predicted thrust with the turbine thrust and monitoring the resulting error and also the limit avoidance parameter, e_dp by checking the turbine thrust force whether staying within the limit or riding the turbine approximately at the limit.

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
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW_AEPS.yaml')
    run_dir = os.path.join(example_out_dir,'34_AEPS_Simulation')
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
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U30.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U32.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U33.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U34.000000_Seed1501552846.0.bts'],
         #'wind_filenames': ['C:/Users/musah/ROSCO/Examples/Test_Cases/IEA-15-240-RWT/IEA-15-240-RWT/Wind/testbench_NTM_U40.000000_Seed1501552846.0.bts'],
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
            'param_values': [0,1] #Run with ASO mode equal to 0 and 1, respectively.
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
