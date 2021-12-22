"""

Example script to run the DLCs in OpenFAST

"""

from ROSCO_toolbox.ofTools.case_gen.runFAST_pywrapper   import runFAST_pywrapper, runFAST_pywrapper_batch
from ROSCO_toolbox.ofTools.case_gen.CaseGen_IEC         import CaseGen_IEC
from ROSCO_toolbox.ofTools.case_gen.CaseGen_General     import CaseGen_General
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
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
rosco_dir       = os.path.realpath(os.path.join(this_dir,'../../..'))

class run_FAST_ROSCO():

    def __init__(self):

        # Set default parameters
        self.tuning_yaml        = os.path.join(tune_case_dir,'IEA15MW.yaml')
        self.wind_case_fcn      = cl.power_curve
        self.wind_case_opts     = {}
        self.control_sweep_fcn  = None
        self.save_dir           = os.path.join(rosco_dir,'outputs')
        self.n_cores            = 1

    def run_FAST(self):
        # set up run directory
        if self.control_sweep_fcn:
            sweep_name = self.control_sweep_fcn.__name__
        else:
            sweep_name = 'base'

        turbine_name = os.path.split(self.tuning_yaml)[-1].split('.')[0]
        run_dir = os.path.join(self.save_dir,turbine_name,self.wind_case_fcn.__name__,sweep_name)

        
        # Start with tuning yaml definition of controller
        if not os.path.isabs(self.tuning_yaml):
            self.tuning_yaml = os.path.join(tune_case_dir,self.tuning_yaml)


        # Load yaml file 
        inps = load_rosco_yaml(self.tuning_yaml)
        path_params         = inps['path_params']
        turbine_params      = inps['turbine_params']
        controller_params   = inps['controller_params']

        # Instantiate turbine, controller, and file processing classes
        turbine         = ROSCO_turbine.Turbine(turbine_params)
        controller      = ROSCO_controller.Controller(controller_params)

        # Load turbine data from OpenFAST and rotor performance text file
        cp_filename = os.path.join(tune_case_dir,path_params['rotor_performance_filename'])
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
        case_inputs = self.wind_case_fcn(run_dir)
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
        if self.control_sweep_fcn:
            case_inputs_control = self.control_sweep_fcn(self.tuning_yaml,cl.find_max_group(case_inputs)+1)
            sweep_name = self.control_sweep_fcn.__name__
            case_inputs.update(case_inputs_control)
        else:
            sweep_name = 'base'

        
            
        # Generate cases
        case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase=turbine_name)
        channels = cl.set_channels()

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
                if self.n_cores == 1:
                    fastBatch.run_serial()
                else:
                    fastBatch.run_multi(cores=self.n_cores)

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
    sim_config = 1
    
    r = run_FAST_ROSCO()

    wind_case_opts = {}

    if sim_config == 1:
        # FOCAL single wind speed testing
        r.tuning_yaml = os.path.join(tune_case_dir,'IEA15MW.yaml')
        r.wind_case   = cl.simp_step
        r.sweep_mode  = None
        r.save_dir    = '/Users/dzalkind/Tools/ROSCO/outputs'
    
    elif sim_config == 6:

        # FOCAL rated wind speed tuning
        r.tuning_yaml = os.path.join(tune_case_dir,'IEA15MW_FOCAL.yaml')
        r.wind_case   = power_curve
        r.sweep_mode  = cl.sweep_rated_torque
        r.save_dir    = '/Users/dzalkind/Projects/FOCAL/drop_torque'

    elif sim_config == 7:

        # FOCAL rated wind speed tuning
        r.tuning_yaml = os.path.join(tune_case_dir,'IEA15MW.yaml')
        r.wind_case   = cl.steps
        r.wind_case_opts = {
            'times': [200,300,400,500], 
            'winds': [6,10,14,18]
            }
        r.sweep_mode  = None
        r.save_dir    = '/Users/dzalkind/Tools/ROSCO/outputs'

    else:
        raise Exception('This simulation configuration is not supported.')


    r.run_FAST()


    
    
    
    
