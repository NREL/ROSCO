'''
----------- 26_marine_hydro ------------------------
Run openfast with ROSCO and a MHK turbine
-----------------------------------------------


'''

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml
import matplotlib.pyplot as plt

'''
Run MHK turbine in OpenFAST with ROSCO torque controller


'''


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/RM1_MHK_FBP.yaml')
    run_dir = os.path.join(example_out_dir,'29_MHK/0_baseline')
    os.makedirs(run_dir,exist_ok=True)

    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(this_dir,path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(this_dir,path_params['FAST_directory']),
        rot_source='txt',txt_filename= cp_filename
        )

    # Tune controller 
    controller.tune_controller(turbine)

    # Write parameter input file
    param_file = os.path.join(run_dir,'DISCON.IN')
    write_DISCON(turbine,
                 controller,
                 param_file=param_file, 
                 txt_filename=cp_filename
    )

    # # Plot operating schedule
    # fig, ax = plt.subplots(2,2,constrained_layout=True,sharex=True)
    # ax = ax.flatten()
    # ax[0].plot(controller.v[len(controller.v_below_rated)+1:], controller.omega_pc_U)
    # ax[0].set_ylabel('omega_pc')

    # ax[1].plot(controller.v[len(controller.v_below_rated)+1:], controller.zeta_pc_U)
    # ax[1].set_ylabel('zeta_pc')

    # ax[2].plot(controller.v[len(controller.v_below_rated)+1:], controller.pc_gain_schedule.Kp)
    # ax[2].set_xlabel('Wind Speed')
    # ax[2].set_ylabel('Proportional Gain')

    # ax[3].plot(controller.v[len(controller.v_below_rated)+1:], controller.pc_gain_schedule.Ki)
    # ax[3].set_xlabel('Wind Speed')
    # ax[3].set_ylabel('Integral Gain')

    # plt.suptitle('Pitch Controller Gains')


    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    # r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [3.5],
        'TMax': 100.0,
        }
    r.case_inputs = {}
    # r.fst_vt        = reader.fst_vt
    # r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()

    op = output_processing.output_processing()
    fast_out = op.load_fast_out([os.path.join(run_dir,'RM1_MHK_FBP/power_curve/base/RM1_MHK_FBP_0.out')], tmin=0)
    fig, axs = plt.subplots(4,1)
    axs[0].plot(fast_out[0]['Time'],fast_out[0]['Wind1VelX'],label='Flow Speed')
    axs[1].plot(fast_out[0]['Time'],fast_out[0]['GenSpeed'],label='Gen Speed')
    axs[2].plot(fast_out[0]['Time'],fast_out[0]['GenTq'],label='Gen Torque')
    axs[3].plot(fast_out[0]['Time'],fast_out[0]['GenPwr'],label='Gen Power')


    # op = output_processing.output_processing()
    # op2 = output_processing.output_processing()

    # md_out = op.load_fast_out([os.path.join(run_dir,'IEA15MW_cable/power_curve/base/IEA15MW_cable_0.MD.Line1.out')], tmin=0)
    # local_vars = op2.load_fast_out([os.path.join(run_dir,'IEA15MW_cable/power_curve/base/IEA15MW_cable_0.RO.dbg2')], tmin=0)

    # fig, axs = plt.subplots(4,1)
    # axs[0].plot(local_vars[0]['Time'],local_vars[0]['CC_DesiredL'],label='CC_DesiredL')
    # axs[1].plot(local_vars[0]['Time'],local_vars[0]['CC_ActuatedL'],label='CC_ActuatedL')
    # axs[2].plot(local_vars[0]['Time'],local_vars[0]['CC_ActuatedDL'],label='CC_ActuatedDL')
    # axs[3].plot(md_out[0]['Time'],md_out[0]['Seg20Lst'],label='Seg20Lst')

    # [a.legend() for a in axs]
    # [a.grid() for a in axs]

    # if False:
    #     plt.show()
    # else:
    #     plt.savefig(os.path.join(example_out_dir,'22_cable_control.png'))

    # # Check that the last segment of line 1 shrinks by 10 m
    # # np.testing.assert_almost_equal(md_out[0]['Seg20Lst'][-1] - md_out[0]['Seg20Lst'][0], line_ends[0], 2)



if __name__=="__main__":
    main()
