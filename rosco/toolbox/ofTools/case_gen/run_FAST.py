"""

Example script to run the DLCs in OpenFAST

This script is designed to work as-is if ROSCO is installed in 'develop' mode, i.e., python setup.py develop
Otherwise, the directories can be defined as attributes of the run_FAST_ROSCO

"""
import sys
import os
import platform
#import pickle
import collections.abc
import numpy as np
from rosco import discon_lib_path
from rosco.toolbox import utilities as ROSCO_utilities
from rosco.toolbox.inputs.validation import load_rosco_yaml

from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine

try:
    from weis.aeroelasticse.runFAST_pywrapper   import runFAST_pywrapper_batch
    in_weis = True
except Exception as e:
    from rosco.toolbox.ofTools.case_gen.runFAST_pywrapper   import runFAST_pywrapper_batch
    in_weis = False
#from rosco.toolbox.ofTools.case_gen.CaseGen_IEC         import CaseGen_IEC
from rosco.toolbox.ofTools.case_gen.CaseGen_General     import CaseGen_General
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from openmdao.utils.mpi              import MPI

# Globals
this_dir        = os.path.dirname(os.path.abspath(__file__))

# https://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth
def update_deep(d, u):
    for k, v in u.items():
        if isinstance(v, collections.abc.Mapping):
            d[k] = update_deep(d.get(k, {}), v)
        else:
            d[k] = v
    return d

class run_FAST_ROSCO():

    def __init__(self):

        # Set default parameters
        self.tuning_yaml        = 'IEA15MW.yaml'
        self.wind_case_fcn      = cl.power_curve
        self.wind_case_opts     = {}
        self.control_sweep_opts = {}
        self.control_sweep_fcn  = None
        self.case_inputs        = {}
        self.rosco_dll          = ''
        self.n_cores            = 1
        self.base_name          = ''
        self.controller_params  = {}   
        self.fst_vt             = {}   
        self.openfast_exe       = 'openfast'

        # Directories
        self.tune_case_dir  = ''
        self.rosco_dir      = ''
        self.save_dir       = ''

    def run_FAST(self):

        # handle directories, set defaults
        if not self.rosco_dir:
            # Top ROSCO directory of whole repo
            self.rosco_dir = os.path.realpath(os.path.join(this_dir,'../../../..'))

        if not self.tune_case_dir:
            self.tune_case_dir = os.path.realpath(os.path.join(self.rosco_dir,'Examples/Tune_Cases'))

        if not self.save_dir:
            self.save_dir       = os.path.join(self.rosco_dir,'outputs')


        # set up run directory
        if self.control_sweep_fcn:
            sweep_name = self.control_sweep_fcn.__name__
        else:
            sweep_name = 'base'

        # Base name and run directory
        if not self.base_name:
            self.base_name = os.path.split(self.tuning_yaml)[-1].split('.')[0]
        
        run_dir = os.path.join(self.save_dir,self.base_name,self.wind_case_fcn.__name__,sweep_name)

        
        # Start with tuning yaml definition of controller
        if not os.path.isabs(self.tuning_yaml):
            self.tuning_yaml = os.path.join(self.tune_case_dir,self.tuning_yaml)

        # Load yaml file 
        inps = load_rosco_yaml(self.tuning_yaml)
        path_params         = inps['path_params']
        turbine_params      = inps['turbine_params']
        controller_params   = inps['controller_params']

        # Update user-defined controller_params
        update_deep(controller_params,self.controller_params)

        # Instantiate turbine, controller, and file processing classes
        turbine         = ROSCO_turbine.Turbine(turbine_params)
        controller      = ROSCO_controller.Controller(controller_params)

        # Load turbine data from OpenFAST and rotor performance text file
        tune_yaml_dir = os.path.split(self.tuning_yaml)[0]
        cp_filename = os.path.join(
            tune_yaml_dir,
            path_params['rotor_performance_filename']
            )
        turbine.load_from_fast(
            path_params['FAST_InputFile'],
            os.path.join(tune_yaml_dir,path_params['FAST_directory']),
            rot_source='txt',
            txt_filename=cp_filename
            )

        # tune base controller defined by the yaml
        controller.tune_controller(turbine)

        # Apply all discon variables as case inputs
        discon_vt = ROSCO_utilities.DISCON_dict(turbine, controller, txt_filename=cp_filename)
        control_base_case = {}
        for discon_input in discon_vt:
            control_base_case[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

        # Set up wind case
        self.wind_case_opts['run_dir'] = run_dir
        case_inputs = self.wind_case_fcn(**self.wind_case_opts)
        case_inputs.update(control_base_case)

        if not self.rosco_dll:
            self.rosco_dll = discon_lib_path

        case_inputs[('ServoDyn','DLL_FileName')] = {'vals': [self.rosco_dll], 'group': 0}

        # Sweep control parameter
        if self.control_sweep_fcn:
            self.control_sweep_opts['tuning_yaml'] = self.tuning_yaml
            self.control_sweep_opts['controller_params'] = controller_params
            case_inputs_control = self.control_sweep_fcn(cl.find_max_group(case_inputs)+1, **self.control_sweep_opts)
            sweep_name = self.control_sweep_fcn.__name__
            case_inputs.update(case_inputs_control)
        else:
            sweep_name = 'base'

        # Add external user-defined case inputs
        case_inputs.update(self.case_inputs)
            
        # Generate cases
        case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase=self.base_name)
        channels = cl.set_channels()

        # Management of parallelization, leave in for now
        if MPI:
            from rosco.toolbox.ofTools.util.mpi_tools import map_comm_heirarchical, subprocessor_loop, subprocessor_stop
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
            fastBatch                   = runFAST_pywrapper_batch()
            
            # FAST_directory (relative to Tune_Dir/)
            fastBatch.FAST_directory    = os.path.realpath(os.path.join(tune_yaml_dir,path_params['FAST_directory']))
            fastBatch.FAST_InputFile    = path_params['FAST_InputFile']        
            fastBatch.channels          = channels
            fastBatch.FAST_runDirectory = run_dir
            fastBatch.case_list         = case_list
            fastBatch.fst_vt            = self.fst_vt
            fastBatch.case_name_list    = case_name_list
            fastBatch.FAST_exe          = self.openfast_exe
            fastBatch.use_exe           = True

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
    sim_config = 8
    
    r = run_FAST_ROSCO()

    wind_case_opts = {}

    if sim_config == 1:
        # FOCAL single wind speed testing
        r.tuning_yaml = 'IEA15MW.yaml'
        r.wind_case_fcn = cl.simp_step
        r.save_dir    = '/Users/dzalkind/Tools/ROSCO/outputs'

    elif sim_config == 3:
        # IEA-22 fixed bottom
        r.tuning_yaml = '/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/OpenFAST/IEA-22-280-RWT-Monopile/IEA-22-280-RWT-Monopile.yaml'
        r.wind_case_fcn = cl.power_curve
        r.wind_case_opts    = {
            'U': [6,9,12,15],
            'TMax': 100,
            }
        r.save_dir    = '/Users/dzalkind/Projects/IEA-22MW/MonopileControl/outputs/1_const_power'
        r.openfast_exe = '/Users/dzalkind/opt/anaconda3/envs/rosco-new/bin/openfast'
        # rosco_dll = '/Users/dzalkind/Tools/ROSCO1/ROSCO/build/libdiscon.dylib'
        r.case_inputs = {}
        # r.case_inputs[('ServoDyn','DLL_FileName')] = {'vals': [rosco_dll], 'group': 0}
        r.n_cores = 4
    
    elif sim_config == 6:

        # FOCAL rated wind speed tuning
        r.tuning_yaml   = 'IEA15MW_FOCAL.yaml'
        r.wind_case_fcn = cl.power_curve
        r.sweep_mode    = cl.sweep_rated_torque
        r.save_dir      = '/Users/dzalkind/Projects/FOCAL/drop_torque'

    elif sim_config == 7:

        # FOCAL rated wind speed tuning
        r.tuning_yaml   = 'IEA15MW.yaml'
        r.wind_case_fcn     = cl.steps
        r.wind_case_opts    = {
            'tt': [100,200], 
            'U': [16,18],
            'U_0': 13
            }
        r.sweep_mode    = None
        r.save_dir      = '/Users/dzalkind/Tools/ROSCO/outputs'
        r.control_sweep_fcn = cl.sweep_pitch_act
        r.control_sweep_opts = {
            'act_bw': np.array([0.25,0.5,1,10]) * np.pi * 2
        }

        r.n_cores = 4

    elif sim_config == 8:

        # FOCAL rated wind speed tuning
        r.tuning_yaml   = 'IEA15MW.yaml'
        r.wind_case_fcn     = cl.simp_step
        r.wind_case_opts    = {
            'U_start': [7],
            'U_end': [9],
            }
        r.sweep_mode    = None
        r.save_dir      = '/Users/dzalkind/Tools/ROSCO1/outputs/VS_Mode_2'
        r.control_sweep_fcn = cl.sweep_yaml_input
        r.control_sweep_opts = {
            'control_param': 'VS_ControlMode',
            'param_values': [2,3]
        }
        # r.controller_params = {'VS_ControlMode': 3}

        r.n_cores = 2

    else:
        raise Exception('This simulation configuration is not supported.')


    r.run_FAST()


    
    
    
    
