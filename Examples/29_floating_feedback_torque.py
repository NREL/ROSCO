'''
----------- 28_floating_feedback_torque ------------------------
Run openfast with ROSCO and torque floating feedback
-----------------------------------------------

Floating feedback methods available in ROSCO/ROSCO_Toolbox

1. Automated tuning, constant for all wind speeds
2. Automated tuning, varies with wind speed
3. Direct tuning, constant for all wind speeds
4. Direct tuning, varies with wind speeds

'''

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
import numpy as np
#from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
#from rosco.toolbox.inputs.validation import load_rosco_yaml
#from rosco.toolbox.controller import OpenLoopControl
from rosco.toolbox.tune import yaml_to_objs
from rosco.toolbox.utilities import write_DISCON, read_DISCON
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox.ofTools.fast_io import output_processing
import matplotlib.pyplot as plt



#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'29_floating_feedback_torque')
    os.makedirs(run_dir,exist_ok=True)

    controller, turbine, path_params = yaml_to_objs(parameter_filename)

    # First, let's write the DISCONs for each method
    param_files = []

    # Method 1: Automated tuning, constant for all wind speeds (default param = 0.5)
    controller_params_1 = controller.controller_params      # numbers correspond to methods above
    controller_params_1['Fl_Mode']   = 0
    controller_params_1['FlTq_Mode'] = 2
    controller_params_1['U_Fl']      = 'all'
    controller_params_1['tune_Fl']   = True
    controller_params_1['tune_FlTq']   = True
    controller = ROSCO_controller.Controller(controller_params_1)
    controller.tune_controller(turbine)
    # Reduce gain
    # controller.Kp_floatTq = controller.Kp_floatTq/2
    param_file = os.path.join(run_dir,'DISCON_Fl_1.IN')
    param_files.append(param_file)
    write_DISCON(turbine,controller,
                 param_file=param_file, 
                 txt_filename=path_params['rotor_performance_filename'])

    # Automated tuning, constant for all wind speeds, with smaller scaling param
    controller_params_2 = controller.controller_params      # numbers correspond to methods above
    controller_params_2['Fl_Mode']   = 0
    controller_params_2['FlTq_Mode'] = 2
    controller_params_2['U_Fl']      = 'all'
    controller_params_2['tune_Fl']   = True
    controller_params_2['tune_FlTq']   = True
    controller_params_2['FlTq_param']   = [0.2]
    controller = ROSCO_controller.Controller(controller_params_1)
    controller.tune_controller(turbine)
    param_file = os.path.join(run_dir,'DISCON_Fl_2.IN')
    param_files.append(param_file)
    write_DISCON(turbine,controller,
                 param_file=param_file, 
                 txt_filename=path_params['rotor_performance_filename'])


    # Automated tuning, constant for all wind speeds, with smaller scaling param
    controller_params_3 = controller.controller_params      # numbers correspond to methods above
    controller_params_3['Fl_Mode']   = 0
    controller_params_3['FlTq_Mode'] = 2
    controller_params_3['U_Fl']      = 'all'
    controller_params_3['tune_Fl']   = True
    controller_params_3['tune_FlTq']   = True
    controller_params_3['FlTq_param']   = [0.8]
    controller = ROSCO_controller.Controller(controller_params_1)
    controller.tune_controller(turbine)
    param_file = os.path.join(run_dir,'DISCON_Fl_3.IN')
    param_files.append(param_file)
    write_DISCON(turbine,controller,
                 param_file=param_file, 
                 txt_filename=path_params['rotor_performance_filename'])


    # Read all DISCONs and make into case_inputs
    case_inputs = {}
    discon_lists = {}  
    for discon in param_files:
        discon_vt = read_DISCON(discon)
        for discon_input in discon_vt:
            if discon_input not in discon_lists:        # initialize
                discon_lists[discon_input] = []
            discon_lists[discon_input].append(discon_vt[discon_input])

    for discon_input, input in discon_lists.items():
        case_inputs[('DISCON_in',discon_input)] = {'vals': input, 'group': 2}


    # Additional config for the torque controller
    case_inputs[('DISCON_in', 'VS_MaxTq')] = {'vals': [turbine.rated_torque * 1.5], 'group': 0} # 29436069.9996
    case_inputs[('ElastoDyn', 'RotSpeed')] = {'vals': [7.56], 'group': 0}
    case_inputs[('ElastoDyn', 'BlPitch1')] = {'vals': [10], 'group': 0}
    case_inputs[('ElastoDyn', 'BlPitch2')] = {'vals': [10], 'group': 0}
    case_inputs[('ElastoDyn', 'BlPitch3')] = {'vals': [10], 'group': 0}

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_opts    = {
        'U_start': [13],
        'U_end': [16],
        'TMax': 400,
        'TStep': 200,
        }
    r.case_inputs       = case_inputs
    r.save_dir          = run_dir
    r.rosco_dir         = rosco_dir
    r.n_cores           = 4
    r.run_FAST()

    op = output_processing.output_processing()
    op_dbg = output_processing.output_processing()
    op_dbg2 = output_processing.output_processing()

    out_files = [os.path.join(run_dir,f'IEA15MW/simp_step/base/IEA15MW_{i_case}.outb') for i_case in range(len(param_files))]
    dbg_files = [os.path.join(run_dir,f'IEA15MW/simp_step/base/IEA15MW_{i_case}.RO.dbg') for i_case in range(len(param_files))]
    dbg2_files = [os.path.join(run_dir,f'IEA15MW/simp_step/base/IEA15MW_{i_case}.RO.dbg2') for i_case in range(len(param_files))]

    fst_out = op.load_fast_out(out_files, tmin=0)
    debug_vars = op_dbg.load_fast_out(dbg_files, tmin=0)
    local_vars = op_dbg2.load_fast_out(dbg2_files, tmin=0)

    comb_out = [None] * len(fst_out)
    for i, (r_out, f_out) in enumerate(zip(debug_vars,fst_out)):
        r_out.update(f_out)
        comb_out[i] = r_out
    for i, (r_out2, f_out) in enumerate(zip(local_vars,comb_out)):
        r_out2.update(f_out)
        comb_out[i] = r_out2

    cases = {}
    cases['Fl Sigs.'] = ['Wind1VelX', 'GenSpeed', 'BldPitch1', 'PtfmPitch', 'Fl_TqCom', 'GenTq'] # , 'NcIMURAys']#,'PtfmPitch','PtfmYaw','NacYaw']
    fig, ax = op.plot_fast_out(comb_out,cases, showplot=False)

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(run_dir,'29_floating_feedback_torque.png'))

    plt.show()

if __name__=="__main__":
    main()
