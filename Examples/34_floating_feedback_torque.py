'''
----------- 29_floating_feedback_torque ------------------------
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
from rosco.toolbox.utilities import DISCON_dict, write_DISCON, read_DISCON
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

    # # First, let's write the DISCONs for each method
    # param_files = []

    # # Method 1: Automated tuning, constant for all wind speeds (default param = 0.5)
    # controller_params_1 = controller.controller_params      # numbers correspond to methods above
    # controller_params_1['Fl_Mode']   = 0
    # controller_params_1['FlTq_Mode'] = 2
    # controller_params_1['U_Fl']      = 'all'
    # controller_params_1['tune_Fl']   = True
    # controller_params_1['tune_FlTq']   = True
    # controller_params_1['FlTq_alpha']   = [0.2] # Low tuning
    # controller_new = ROSCO_controller.Controller(controller_params_1)
    # controller_new.tune_controller(turbine)
    # param_file = os.path.join(run_dir,'DISCON_Fl_1.IN')
    # param_files.append(param_file)
    # write_DISCON(turbine,controller_new,
    #              param_file=param_file, 
    #              txt_filename=path_params['rotor_performance_filename'])

    # # Automated tuning, constant for all wind speeds, with smaller scaling param
    # controller_params_2 = controller.controller_params      # numbers correspond to methods above
    # controller_params_2['Fl_Mode']   = 0
    # controller_params_2['FlTq_Mode'] = 2
    # controller_params_2['U_Fl']      = 'all'
    # controller_params_2['tune_Fl']   = True
    # controller_params_2['tune_FlTq']   = True
    # controller_params_2['FlTq_alpha']   = [0.5] # Moderate tuning
    # controller_new = ROSCO_controller.Controller(controller_params_2)
    # controller_new.tune_controller(turbine)
    # param_file = os.path.join(run_dir,'DISCON_Fl_2.IN')
    # param_files.append(param_file)
    # write_DISCON(turbine,controller_new,
    #              param_file=param_file, 
    #              txt_filename=path_params['rotor_performance_filename'])

    # # Automated tuning, constant for all wind speeds, with smaller scaling param
    # controller_params_3 = controller_new.controller_params      # numbers correspond to methods above
    # controller_params_3['Fl_Mode']   = 0
    # controller_params_3['FlTq_Mode'] = 2
    # controller_params_3['U_Fl']      = 'all'
    # controller_params_3['tune_Fl']   = True
    # controller_params_3['tune_FlTq']   = True
    # controller_params_3['FlTq_alpha']   = [0.8] # Aggressive tuning
    # controller_new = ROSCO_controller.Controller(controller_params_3)
    # controller_new.tune_controller(turbine)
    # param_file = os.path.join(run_dir,'DISCON_Fl_3.IN')
    # param_files.append(param_file)
    # write_DISCON(turbine,controller_new,
    #              param_file=param_file, 
    #              txt_filename=path_params['rotor_performance_filename'])


    # # Read all DISCONs and make into case_inputs
    # case_inputs = {}
    # discon_lists = {}  
    # for discon in param_files:
    #     discon_vt = read_DISCON(discon)
    #     for discon_input in discon_vt:
    #         if discon_input not in discon_lists:        # initialize
    #             discon_lists[discon_input] = []
    #         discon_lists[discon_input].append(discon_vt[discon_input])

    # for discon_input, input in discon_lists.items():
    #     case_inputs[('DISCON_in',discon_input)] = {'vals': input, 'group': 2}


    # # Additional config for the torque controller
    # case_inputs[('DISCON_in', 'VS_MaxTq')] = {'vals': [turbine.rated_torque * 1.5], 'group': 0} # 29436069.9996
    # case_inputs[('ElastoDyn', 'RotSpeed')] = {'vals': [7.56], 'group': 0}
    # case_inputs[('ElastoDyn', 'BlPitch1')] = {'vals': [10], 'group': 0}
    # case_inputs[('ElastoDyn', 'BlPitch2')] = {'vals': [10], 'group': 0}
    # case_inputs[('ElastoDyn', 'BlPitch3')] = {'vals': [10], 'group': 0}

    # # simulation set up
    # r = run_FAST_ROSCO()
    # r.tuning_yaml   = parameter_filename
    # r.wind_case_fcn = cl.simp_step  # single step wind input
    # r.wind_case_opts    = {
    #     'U_start': [13],
    #     'U_end': [16],
    #     'TMax': 800,
    #     'TStep': 400,
    #     }
    # r.case_inputs       = case_inputs
    # r.save_dir          = run_dir
    # r.rosco_dir         = rosco_dir
    # r.n_cores           = 4
    # r.run_FAST()

    # op = output_processing.output_processing()
    # op_dbg = output_processing.output_processing()
    # op_dbg2 = output_processing.output_processing()

    # out_files = [os.path.join(run_dir,f'IEA15MW/simp_step/base/IEA15MW_{i_case}.outb') for i_case in range(len(param_files))]
    # dbg_files = [os.path.join(run_dir,f'IEA15MW/simp_step/base/IEA15MW_{i_case}.RO.dbg') for i_case in range(len(param_files))]
    # dbg2_files = [os.path.join(run_dir,f'IEA15MW/simp_step/base/IEA15MW_{i_case}.RO.dbg2') for i_case in range(len(param_files))]

    # fst_out = op.load_fast_out(out_files, tmin=0)
    # debug_vars = op_dbg.load_fast_out(dbg_files, tmin=0)
    # local_vars = op_dbg2.load_fast_out(dbg2_files, tmin=0)

    # comb_out = [None] * len(fst_out)
    # for i, (r_out, f_out) in enumerate(zip(debug_vars,fst_out)):
    #     r_out.update(f_out)
    #     comb_out[i] = r_out
    # for i, (r_out2, f_out) in enumerate(zip(local_vars,comb_out)):
    #     r_out2.update(f_out)
    #     comb_out[i] = r_out2

    # cases = {}
    # cases['Fl Sigs.'] = ['Wind1VelX', 'GenSpeed', 'GenPwr', 'BldPitch1', 'PtfmPitch', 'NacIMU_FA_AccF', 'NacIMU_FA_AccFT', 'Fl_TqCom', 'GenTq'] # , 'NcIMURAys']#,'PtfmPitch','PtfmYaw','NacYaw']
    # fig, ax = op.plot_fast_out(comb_out, cases, showplot=False)

    # if False:
    #     plt.show()
    # else:
    #     plt.savefig(os.path.join(run_dir,'29_floating_feedback_torque.png'))

    
    # More comprehensive parameter sweep test
    param_files = []

    DISCON_i = 0

    # # Baseline (zero FF)
    # controller_params_new = controller.controller_params
    # controller_params_new['Fl_Mode']   = 0
    # controller_params_new['FlTq_Mode'] = 0
    # controller_params_new['U_Fl']      = []
    # controller_params_new['tune_Fl']   = 0
    # controller_params_new['tune_FlTq']   = 0
    # controller_new = ROSCO_controller.Controller(controller_params_new)
    # controller_new.tune_controller(turbine)
    # param_file = os.path.join(run_dir,'DISCON_Fl_{}.IN'.format(DISCON_i))
    # param_files.append(param_file)
    # write_DISCON(turbine,controller_new,
    #             param_file=param_file, 
    #             txt_filename=path_params['rotor_performance_filename'])

    # DISCON_i += 1

    alpha_tq_cases = np.arange(0, 2.2, .2)
    # alpha_bp_cases = np.arange(0, 2.2, .2)
    alpha_bp_cases = np.arange(0, 1.1, .1)
    alpha_case_matrix_indices = np.zeros([len(alpha_tq_cases), len(alpha_bp_cases)])

    for alpha_tq_ii in range(len(alpha_tq_cases)):

        for alpha_bp_ii in range(len(alpha_bp_cases)):

            controller_params_new = controller.controller_params
            controller_params_new['Fl_Mode']   = 2
            controller_params_new['FlTq_Mode'] = 2
            controller_params_new['U_Fl']      = []
            controller_params_new['tune_Fl']   = 2
            controller_params_new['tune_FlTq'] = 1
            # controller_params_new['Fl_alpha']  = [alpha_bp_cases[alpha_bp_ii]]
            controller_params_new['Fl_Dzeta']  = [alpha_bp_cases[alpha_bp_ii]]
            controller_params_new['FlTq_alpha']  = [alpha_tq_cases[alpha_tq_ii]]
            controller_new = ROSCO_controller.Controller(controller_params_new)
            controller_new.tune_controller(turbine)
            param_file = os.path.join(run_dir,'DISCON_Fl_{}.IN'.format(DISCON_i))
            param_files.append(param_file)
            # Adjust filter frequency manually
            rosco_vt = DISCON_dict(turbine, controller_new, txt_filename=path_params['rotor_performance_filename'])
            rosco_vt['F_FlCornerFreq'] = rosco_vt['F_FlTqCornerFreq']
            write_DISCON(turbine,controller_new,
                        param_file=param_file, 
                        txt_filename=path_params['rotor_performance_filename'])

            # Record index in case matrix
            alpha_case_matrix_indices[alpha_tq_ii, alpha_bp_ii] = DISCON_i

            DISCON_i += 1

    # Dzeta_select = np.arange(.1, .5, .1)
    # for Dzeta in Dzeta_select:

    #     controller_params_new = controller.controller_params
    #     controller_params_new['Fl_Mode']   = 2
    #     controller_params_new['FlTq_Mode'] = 0
    #     controller_params_new['U_Fl']      = []
    #     controller_params_new['tune_Fl']   = 2
    #     controller_params_new['tune_FlTq'] = 0
    #     controller_params_new['Fl_Dzeta']  = [Dzeta]
    #     controller_new = ROSCO_controller.Controller(controller_params_new)
    #     controller_new.tune_controller(turbine)
    #     param_file = os.path.join(run_dir,'DISCON_Fl_{}.IN'.format(DISCON_i))
    #     param_files.append(param_file)
    #     # Adjust filter frequency manually
    #     rosco_vt = DISCON_dict(turbine, controller_new, txt_filename=path_params['rotor_performance_filename'])
    #     rosco_vt['F_FlCornerFreq'] = rosco_vt['F_FlTqCornerFreq']
    #     write_DISCON(turbine,controller_new,
    #                 param_file=param_file, 
    #                 txt_filename=path_params['rotor_performance_filename'])

    #     DISCON_i += 1


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
    # case_inputs[('DISCON_in', 'F_FlCornerFreq')] = {'vals': [], 'group': 0}
    case_inputs[('ElastoDyn', 'RotSpeed')] = {'vals': [7.56], 'group': 0}
    case_inputs[('ElastoDyn', 'BlPitch1')] = {'vals': [10], 'group': 0}
    case_inputs[('ElastoDyn', 'BlPitch2')] = {'vals': [10], 'group': 0}
    case_inputs[('ElastoDyn', 'BlPitch3')] = {'vals': [10], 'group': 0}

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.turb_bts  # single step wind input
    r.wind_case_opts    = {
        'TMax': 720,
        'wind_filenames': ['/home/david/WEIS_clean/ROSCO_fork/rosco/test/testing/wind/IEA-15MW_NTM_U15.000000_Seed971231.0.bts']
        }
    r.case_inputs       = case_inputs
    r.save_dir          = run_dir
    r.rosco_dir         = rosco_dir
    r.n_cores           = 48
    r.run_FAST()

    op = output_processing.output_processing()
    op_dbg = output_processing.output_processing()
    op_dbg2 = output_processing.output_processing()

    out_files = [os.path.join(run_dir,f'IEA15MW/turb_bts/base/IEA15MW_{i_case:03}.outb') for i_case in range(len(param_files))]
    dbg_files = [os.path.join(run_dir,f'IEA15MW/turb_bts/base/IEA15MW_{i_case:03}.RO.dbg') for i_case in range(len(param_files))]
    dbg2_files = [os.path.join(run_dir,f'IEA15MW/turb_bts/base/IEA15MW_{i_case:03}.RO.dbg2') for i_case in range(len(param_files))]

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
    cases['Fl Sigs.'] = ['Wind1VelX', 'GenSpeed', 'GenPwr', 'BldPitch1', 'PtfmPitch', 'Fl_PitCom', 'Fl_TqCom', 'GenTq'] # 'NacIMU_FA_AccF', 'NacIMU_FA_AccFT', 'NcIMURAys']#,'PtfmPitch','PtfmYaw','NacYaw']
    fig, ax = op.plot_fast_out(comb_out, cases, showplot=False)

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(run_dir,'29_floating_feedback_torque_sweep.png'))

    # Compute performance statistics
    # tau_comp_genspeed_data = [comb_out[i]['GenSpeed'] for i in range(6)]
    # tau_comp_power_data = [comb_out[i]['GenPwr'] for i in range(6)]
    # tau_comp_pitch_data = [comb_out[i]['PtfmPitch'] for i in range(6)]
    # tau_comp_genspeed_std = np.std(tau_comp_genspeed_data, axis=1)
    # tau_comp_power_std = np.std(tau_comp_power_data, axis=1)
    # tau_comp_pitch_std = np.std(tau_comp_pitch_data, axis=1)
    # beta_comp_genspeed_data = [comb_out[i]['GenSpeed'] for i in [0, 6, 7, 8, 9, 10]]
    # beta_comp_power_data = [comb_out[i]['GenPwr'] for i in [0, 6, 7, 8, 9, 10]]
    # beta_comp_pitch_data = [comb_out[i]['PtfmPitch'] for i in [0, 6, 7, 8, 9, 10]]
    # beta_comp_genspeed_std = np.std(beta_comp_genspeed_data, axis=1)
    # beta_comp_power_std = np.std(beta_comp_power_data, axis=1)
    # beta_comp_pitch_std = np.std(beta_comp_pitch_data, axis=1)
    # Combined data
    genspeed_data = [comb_out[i]['GenSpeed'] for i in range(len(comb_out))]
    genpower_data = [comb_out[i]['GenPwr'] for i in range(len(comb_out))]
    ptfmpitch_data = [comb_out[i]['PtfmPitch'] for i in range(len(comb_out))]
    genspeed_std = np.std(genspeed_data, axis=1)
    genpower_std = np.std(genpower_data, axis=1)
    ptfmpitch_std = np.std(ptfmpitch_data, axis=1)

    plt.figure()
    plt.subplot(3, 1, 1)
    plt.grid()
    # plt.plot(np.concatenate(([0], alpha_tq_cases)), tau_comp_genspeed_std)
    # plt.plot(np.concatenate(([0], alpha_bp_cases)), beta_comp_genspeed_std)
    plt.contourf(alpha_bp_cases, alpha_tq_cases, genspeed_std[np.int32(alpha_case_matrix_indices)] / (turbine.rated_rotor_speed * turbine.Ng * 60/(2*np.pi)))
    plt.colorbar()
    plt.title('Gen Speed STD')
    plt.ylabel(r'$\alpha_{\tau,comp}$')

    plt.subplot(3, 1, 2)
    plt.grid()
    # plt.plot(np.concatenate(([0], alpha_tq_cases)), tau_comp_power_std)
    # plt.plot(np.concatenate(([0], alpha_bp_cases)), beta_comp_power_std)
    plt.contourf(alpha_bp_cases, alpha_tq_cases, genpower_std[np.int32(alpha_case_matrix_indices)] / (turbine.rated_power/1.0e3))
    plt.colorbar()
    plt.title('Gen Power STD')
    plt.ylabel(r'$\alpha_{\tau,comp}$')

    plt.subplot(3, 1, 3)
    plt.grid()
    # plt.plot(np.concatenate(([0], alpha_tq_cases)), tau_comp_pitch_std)
    # plt.plot(np.concatenate(([0], alpha_bp_cases)), beta_comp_pitch_std)
    plt.contourf(alpha_bp_cases, alpha_tq_cases, ptfmpitch_std[np.int32(alpha_case_matrix_indices)])
    plt.colorbar()
    plt.title('Ptfm Pitch STD')
    plt.ylabel(r'$\alpha_{\tau,comp}$')
    plt.xlabel(r'$\alpha_{\beta,comp}$')

    plt.savefig(os.path.join(run_dir,'29_floating_feedback_torque_sweep_stats.png'))

    plt.show(block=False)
    0

if __name__=="__main__":
    main()
