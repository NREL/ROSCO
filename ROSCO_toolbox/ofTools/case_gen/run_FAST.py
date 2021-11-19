"""

Example script to run the DLCs in OpenFAST

"""

from ROSCO_toolbox.ofTools.case_gen.runFAST_pywrapper   import runFAST_pywrapper, runFAST_pywrapper_batch
from ROSCO_toolbox.ofTools.case_gen.CaseGen_IEC         import CaseGen_IEC
from ROSCO_toolbox.ofTools.case_gen.CaseGen_General     import CaseGen_General
from ROSCO_toolbox.ofTools.case_gen.CaseLibrary         import power_curve, set_channels, find_max_group, sweep_rated_torque, load_tuning_yaml, simp_step
from wisdem.commonse.mpi_tools              import MPI
import sys, os, platform
import numpy as np
from ROSCO_toolbox import utilities as ROSCO_utilities
from ROSCO_toolbox.inputs.validation import load_rosco_yaml

from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine

# Globals
this_dir        = os.path.dirname(os.path.abspath(__file__))
tune_case_dir   = os.path.realpath(os.path.join(this_dir,'../../../Tune_Cases'))


def run_FAST(tuning_yaml,wind_case_fcn,control_sweep_fcn,save_dir,n_cores=1):
    # set up run directory
    if control_sweep_fcn:
        sweep_name = control_sweep_fcn.__name__
    else:
        sweep_name = 'base'

    turbine_name = os.path.split(tuning_yaml)[-1].split('.')[0]
    run_dir = os.path.join(save_dir,turbine_name,wind_case_fcn.__name__,sweep_name)

    
    # Start with tuning yaml definition of controller
    if not os.path.isabs(tuning_yaml):
        tuning_yaml = os.path.join(tune_case_dir,tuning_yaml)


    # Load yaml file 
    inps = load_rosco_yaml(tuning_yaml)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(tune_case_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
    turbine.load_from_fast(path_params['FAST_InputFile'], \
        os.path.join(tune_case_dir,path_params['FAST_directory']), \
        dev_branch=True,rot_source='txt',\
        txt_filename=cp_filename)

    # tune base controller defined by the yaml
    controller.tune_controller(turbine)

    # Apply all discon variables as case inputs
    discon_vt = ROSCO_utilities.DISCON_dict(turbine, controller, txt_filename=cp_filename)
    control_base_case = {}
    for discon_input in discon_vt:
        control_base_case[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

    # Set up wind case
    case_inputs = wind_case_fcn(run_dir)
    case_inputs.update(control_base_case)

    # Specify rosco controller dylib
    rosco_dll = '/Users/dzalkind/Tools/ROSCO/ROSCO/build/libdiscon.dylib' #'/Users/dzalkind/Tools/ROSCO_toolbox/ROSCO/build/libdiscon.dylib'

    if not rosco_dll: # use WEIS ROSCO
        run_dir1            = os.path.dirname( os.path.dirname( os.path.dirname( os.path.realpath(__file__) ) ) ) + os.sep
        if platform.system() == 'Windows':
            rosco_dll = os.path.join(run_dir1, 'local/lib/libdiscon.dll')
        elif platform.system() == 'Darwin':
            rosco_dll = os.path.join(run_dir1, 'local/lib/libdiscon.dylib')
        else:
            rosco_dll = os.path.join(run_dir1, 'local/lib/libdiscon.so')

    case_inputs[('ServoDyn','DLL_FileName')] = {'vals': [rosco_dll], 'group': 0}

    # Sweep control parameter
    if control_sweep_fcn:
        case_inputs_control = control_sweep_fcn(tuning_yaml,find_max_group(case_inputs)+1)
        sweep_name = control_sweep_fcn.__name__
        case_inputs.update(case_inputs_control)
    else:
        sweep_name = 'base'

    
        
    # Generate cases
    case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase=turbine_name)
    channels = set_channels()

    # Management of parallelization, leave in for now
    if MPI:
        from wisdem.commonse.mpi_tools import map_comm_heirarchical, subprocessor_loop, subprocessor_stop
        n_OF_runs = len(case_list)

        available_cores = MPI.COMM_WORLD.Get_size()
        n_parallel_OFruns = np.min([available_cores - 1, n_OF_runs])
        comm_map_down, comm_map_up, color_map = map_comm_heirarchical(1, n_parallel_OFruns)
        sys.stdout.flush()


    # Parallel file generation with MPI
    if MPI:
        comm = MPI.COMM_WORLD
        rank = comm.Get_rank()
    else:
        rank = 0
    if rank == 0:

        # Run FAST cases
        fastBatch                   = runFAST_pywrapper_batch(FAST_ver='OpenFAST',dev_branch = True)
        
        # Select Turbine Model
        model_dir                   = os.path.join(os.path.dirname( os.path.dirname( os.path.realpath(__file__) ) ), '01_aeroelasticse/OpenFAST_models')

        # FAST_directory (relative to Tune_Dir/)
        fastBatch.FAST_directory    = os.path.realpath(os.path.join(tune_case_dir,path_params['FAST_directory']))
        fastBatch.FAST_InputFile    = path_params['FAST_InputFile']        
        fastBatch.channels          = channels
        fastBatch.FAST_runDirectory = run_dir
        fastBatch.case_list         = case_list
        fastBatch.case_name_list    = case_name_list
        fastBatch.debug_level       = 2
        fastBatch.FAST_exe          = 'openfast'

        if MPI:
            fastBatch.run_mpi(comm_map_down)
        else:
            if n_cores == 1:
                fastBatch.run_serial()
            else:
                fastBatch.run_multi(cores=n_cores)

    if MPI:
        sys.stdout.flush()
        if rank in comm_map_up.keys():
            subprocessor_loop(comm_map_up)
        sys.stdout.flush()

    # Close signal to subprocessors
    if rank == 0 and MPI:
        subprocessor_stop(comm_map_down)
    sys.stdout.flush()
    

if __name__ == "__main__":

    # Simulation config
    sim_config = 6
    n_cores = 8

    if sim_config == 1:
        # FOCAL single wind speed testing
        tuning_yaml = '/Users/dzalkind/Tools/ROSCO/Tune_Cases/IEA15MW_FOCAL.yaml'
        wind_case   = simp_step
        sweep_mode  = None
        save_dir    = '/Users/dzalkind/Projects/FOCAL/torque_274'
    
    elif sim_config == 6:

        # FOCAL rated wind speed tuning
        tuning_yaml = '/Users/dzalkind/Tools/ROSCO/Tune_Cases/IEA15MW_FOCAL.yaml'
        wind_case   = power_curve
        sweep_mode  = sweep_rated_torque
        save_dir    = '/Users/dzalkind/Projects/FOCAL/drop_torque'

    else:
        raise Exception('This simulation configuration is not supported.')




    run_FAST(tuning_yaml,wind_case,sweep_mode,save_dir,n_cores=n_cores)

    # # Options: simp, pwr_curve
    # test_type   = 'pwr_curve'

    # save_dir_list    = [os.path.join(res_dir,tm,os.path.basename(dl).split('.')[0],test_type) \
    #     for tm, dl in zip(turbine_mods,discon_list)]

    # for tm, co, sd in zip(turbine_mods,discon_list,save_dir_list):
    #     run_Simp(tm,co,sd,n_cores=8)
    
    
    
    
