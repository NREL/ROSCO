'''
A class for setup and execution of OpenFAST for testing and verification of the ROSCO 
controller. 


Run ROSCO and test against baseline results:
    - set up case(s) using CaseLibrary from aeroelasticse
    - run fast simulations
    - evaluate & compare results with pCrunch
    - report results to user


'''

import numpy as np
import os
import glob
import multiprocessing as mp

from wisdem.aeroelasticse.FAST_reader import InputReader_Common, InputReader_OpenFAST, InputReader_FAST7
from wisdem.aeroelasticse.FAST_writer import InputWriter_Common, InputWriter_OpenFAST, InputWriter_FAST7
from wisdem.aeroelasticse.runFAST_pywrapper import runFAST_pywrapper_batch
from wisdem.aeroelasticse.CaseGen_General import CaseGen_General
from wisdem.aeroelasticse.CaseGen_IEC import CaseGen_IEC
from pCrunch import pdTools
from pCrunch import Processing, Analysis


class ROSCO_testing():
    '''
    Primary ROSCO test scripts
    '''

    def __init__(self, **kwargs):


        # Setup simulation parameters
        self.runDir = 'testruns'    # directory to run simulations in
        self.windDir = None
        self.namebase = 'ROtest'    # root name for output simulations
        self.FAST_exe = 'openfast_single'       # name of openfast executable (may need full path)
        self.Turbsim_exe = 'turbsim_single'     # name of turbsim executable
        self.FAST_ver = 'OpenFAST'  # Fast version
        # Path to ROSCO controller - default to ROSCO Toolbox submodule
        try:
            self.rosco_path = glob.glob(os.path.join(os.path.dirname(os.path.realpath(__file__)),'../ROSCO/build/libdiscon.*'))[0]
        except:
            print('No compiled ROSCO version found, please provide ROSCO_testing.rosco_path.')
        self.dev_branch = True      # openfast dev branch?
        self.debug_level = 2        # debug level. 0 - no outputs, 1 - minimal outputs, 2 - all outputs
        self.overwrite = False      # overwrite existing files? 
        self.cores = 0              # number of cores to use
        self.mpi_run = False
        self.mpi_comm_map_down = []
        self.outfile_fmt = 2 # 1 = .txt, 2 = binary, 3 = both

        # Setup turbine parameters 
        #  - Default to NREL 5MW 
        self.Turbine_Class = 'I'
        self.Turbulence_Class = 'A'
        self.FAST_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../Test_Cases/NREL-5MW')
        self.FAST_InputFile = 'NREL-5MW.fst'

        # Desired output channesl
        self.var_out = [
                        'BldPitch1', 'BldPitch2', 'BldPitch3', 'GenTq', 'GenPwr', 'RotSpeed',
                        'TipDxc1', 'TipDyc1', 'TipDzc1', 'TipDxc2',
                        'TipDyc2', 'TipDzc2', 'TipDxc3', 'TipDyc3', 'TipDzc3',
                        'RootMxc1', 'RootMyc1', 'RootMzc1',
                        'RootMxc2', 'RootMyc2', 'RootMzc2',
                        'RootMxc3', 'RootMyc3', 'RootMzc3',
                        'RootMxb1', 'RootMyb1', 'RootMzb1',
                        'RootMxb2', 'RootMyb2', 'RootMzb2',
                        'RootMxb3', 'RootMyb3', 'RootMzb3',
                        'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt',
                        'TwrBsFxt', 'TwrBsFyt', 'TwrBsFzt',
                        'Wind1VelX', 'Wind1VelY', 'Wind1VelZ',
                        'RtVAvgxh', 'RtVAvgyh', 'RtVAvgzh'
                        ]
                        
        if self.cores > mp.cpu_count():
            self.parallel_cores = mp.cpu_count()

        for (k, w) in kwargs.items():
            try:
                setattr(self, k, w)
            except:
                pass

        super(ROSCO_testing, self).__init__()

    def ROSCO_Test_lite(self, more_case_inputs=None):
        '''
        DLC 1.1 - 5 wind speeds, 60s

        Parameters:
        -----------
        more_case_inputs: dict
            Additional case inputs
        '''
        TMax = 360

        fastRead = InputReader_OpenFAST(
            FAST_ver=self.FAST_ver, dev_branch=self.dev_branch)
        fastRead.FAST_InputFile =  self.FAST_InputFile   # FAST input file (ext=.fst)
        # Path to fst directory files
        fastRead.FAST_directory = self.FAST_directory

        # Read FAST inputs for generating cases
        fastRead.execute()

        # Start near the steady state, controller should be able to handle startup transients.
        iec = CaseGen_IEC()
        iec.init_cond[("ElastoDyn", "RotSpeed")] = {'U':  [2, 30]}
        iec.init_cond[("ElastoDyn", "RotSpeed")]['val'] = np.ones(
            [2]) * fastRead.fst_vt['ElastoDyn']['RotSpeed'] * .75
        iec.init_cond[("ElastoDyn", "BlPitch1")] = {'U':  [2, 30]}
        iec.init_cond[("ElastoDyn", "BlPitch1")]['val'] = np.ones([2]) * 0
        iec.init_cond[("ElastoDyn", "BlPitch2")] = iec.init_cond[("ElastoDyn", "BlPitch1")]
        iec.init_cond[("ElastoDyn", "BlPitch3")] = iec.init_cond[("ElastoDyn", "BlPitch1")]
        iec.Turbine_Class = self.Turbine_Class
        iec.Turbulence_Class = self.Turbulence_Class
        iec.D = fastRead.fst_vt['ElastoDyn']['TipRad']*2.
        iec.z_hub = fastRead.fst_vt['InflowWind']['RefHt']
        iec.TMax = TMax

        iec.dlc_inputs = {}
        iec.dlc_inputs['DLC'] = [1.1]  # ,6.1,6.3]
        iec.dlc_inputs['U'] = [[5, 8, 11, 14, 17]]
        # nothing special about these seeds, randomly generated (???)
        iec.dlc_inputs['Seeds'] = [[11346]]
        iec.dlc_inputs['Yaw'] = [[]]
        iec.transient_dir_change = '-'  # '+','-','both': sign for transient events in EDC, EWS
        iec.transient_shear_orientation = 'v'  # 'v','h','both': vertical or horizontal shear for EWS

        if self.windDir:
            iec.wind_dir = self.windDir
        else:
            iec.wind_dir = os.path.join(self.runDir, 'wind')
            
        iec.case_name_base = self.namebase
        iec.Turbsim_exe = self.Turbsim_exe
        iec.debug_level = self.debug_level
        iec.cores = self.cores
        iec.run_dir = os.path.join(self.runDir,'wind')
        iec.overwrite = self.overwrite
        # iec.overwrite       = False
        if self.cores > 1:
            iec.parallel_windfile_gen = True
        else:
            iec.parallel_windfile_gen = False

        # mpi_run = False
        if self.mpi_run:
            iec.mpi_run = mpi_run
            iec.comm_map_down = mpi_comm_map_down

        case_inputs = {}
        case_inputs[("Fst", "TMax")] = {'vals': [TMax], 'group': 0}
        case_inputs[("Fst", "OutFileFmt")] = {'vals': [self.outfile_fmt], 'group': 0}

        case_inputs[('ServoDyn', 'GenTiStr')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ServoDyn', 'GenTiStp')] = {'vals': ['True'], 'group': 0}
        case_inputs[('ServoDyn', 'SpdGenOn')] = {'vals': [0.], 'group': 0}
        case_inputs[('ServoDyn', 'DLL_FileName')] = {'vals': [self.rosco_path], 'group': 0}

        case_inputs[("AeroDyn15", "WakeMod")] = {'vals': [1], 'group': 0}
        case_inputs[("AeroDyn15", "AFAeroMod")] = {'vals': [2], 'group': 0}
        case_inputs[("AeroDyn15", "TwrPotent")] = {'vals': [0], 'group': 0}
        case_inputs[("AeroDyn15", "TwrShadow")] = {'vals': ['False'], 'group': 0}
        case_inputs[("AeroDyn15", "TwrAero")] = {'vals': ['False'], 'group': 0}
        case_inputs[("AeroDyn15", "SkewMod")] = {'vals': [1], 'group': 0}
        case_inputs[("AeroDyn15", "TipLoss")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "HubLoss")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "TanInd")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "AIDrag")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "TIDrag")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "IndToler")] = {'vals': [1.e-5], 'group': 0}
        case_inputs[("AeroDyn15", "MaxIter")] = {'vals': [5000], 'group': 0}
        case_inputs[("AeroDyn15", "UseBlCm")] = {'vals': ['True'], 'group': 0}

        if more_case_inputs:
            case_inputs.update(more_case_inputs)
        
        # generate cases
        case_list, case_name_list = iec.execute(case_inputs=case_inputs)

        # Ensure proper output channels
        var_out = self.var_out

        channels = {}
        for var in var_out:
            channels[var] = True

        # Set up FAST Sims
        fastBatch = runFAST_pywrapper_batch()
        fastBatch.FAST_ver = self.FAST_ver
        fastBatch.FAST_exe = self.FAST_exe   # Path to executable
        fastBatch.FAST_runDirectory = self.runDir
        fastBatch.FAST_InputFile = self.FAST_InputFile  # FAST input file (ext=.fst)
        fastBatch.FAST_directory = self.FAST_directory   # Path to fst directory files
        fastBatch.debug_level = self.debug_level
        fastBatch.dev_branch = self.dev_branch

        fastBatch.case_list = case_list
        fastBatch.case_name_list = case_name_list
        fastBatch.channels = channels

        # Check if simulation has been run
        if self.outfile_fmt == 1:
            outfile_ext = '.out'
        elif self.outfile_fmt == 2:
            outfile_ext = '.outb'
        elif self.outfile_fmt == 3:
            outfile_ext = '.outb'
        else:
            print('Warning, outfile format may be invalid. Attempting to read binary outputs.')
            outfile_ext = 'outb'
        outFileNames = [os.path.join(fastBatch.FAST_runDirectory, case_name + outfile_ext)
                        for case_name in case_name_list]
        outFileThere = [os.path.exists(outFileName) for outFileName in outFileNames]

        # Run simulations if they're not all there or if you want to overwrite
        if not all(outFileThere) or self.overwrite:
            if self.cores > 1:
                fastBatch.run_multi(self.cores)
            else:
                fastBatch.run_serial()


    def ROSCO_Test_heavy(self, more_case_inputs=None):
        '''
        Run extensive DLCs for ROSCO
            - DLC 1.3 - Cutin-Cutout, 2 seeds
            - DLC 1.4 - 2 wind speeds

        more_case_inputs: dict
            Additional case inputs
        '''

        TMax = 10

        fastRead = InputReader_OpenFAST(
            FAST_ver=self.FAST_ver, dev_branch=self.dev_branch)
        fastRead.FAST_InputFile = self.FAST_InputFile   # FAST input file (ext=.fst)
        # Path to fst directory files
        fastRead.FAST_directory = self.FAST_directory

        # Read FAST inputs for generating cases
        fastRead.execute()

        # Start near the steady state, controller should be able to handle startup transients.
        iec = CaseGen_IEC()
        iec.init_cond[("ElastoDyn", "RotSpeed")] = {'U':  [2, 30]}
        iec.init_cond[("ElastoDyn", "RotSpeed")]['val'] = np.ones(
            [2]) * fastRead.fst_vt['ElastoDyn']['RotSpeed'] * .75
        iec.init_cond[("ElastoDyn", "BlPitch1")] = {'U':  [2, 30]}
        iec.init_cond[("ElastoDyn", "BlPitch1")]['val'] = np.ones([2]) * 0
        iec.init_cond[("ElastoDyn", "BlPitch2")] = iec.init_cond[("ElastoDyn", "BlPitch1")]
        iec.init_cond[("ElastoDyn", "BlPitch3")] = iec.init_cond[("ElastoDyn", "BlPitch1")]
        iec.Turbine_Class = self.Turbine_Class
        iec.Turbulence_Class = self.Turbulence_Class
        iec.D = fastRead.fst_vt['ElastoDyn']['TipRad']*2.
        iec.z_hub = fastRead.fst_vt['InflowWind']['RefHt']
        iec.TMax = TMax

        iec.dlc_inputs = {}
        iec.dlc_inputs['DLC'] = [1.3, 1.4] 
        iec.dlc_inputs['U'] = [[4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], [8.88, 12.88]]
        iec.dlc_inputs['Seeds'] = [[991235, 5123], []]
        iec.dlc_inputs['Yaw'] = [[], []]
        iec.transient_dir_change = '-'  # '+','-','both': sign for transient events in EDC, EWS
        iec.transient_shear_orientation = 'v'  # 'v','h','both': vertical or horizontal shear for EWS

        if self.windDir:
            iec.wind_dir = self.windDir
        else:
            iec.wind_dir = os.path.join(self.runDir, 'wind')
        iec.case_name_base = self.namebase
        iec.Turbsim_exe = self.Turbsim_exe
        iec.debug_level = self.debug_level
        iec.cores = self.cores
        iec.run_dir = os.path.join(self.runDir, 'wind')
        iec.overwrite = self.overwrite
        # iec.overwrite       = False
        if self.cores > 1:
            iec.parallel_windfile_gen = True
        else:
            iec.parallel_windfile_gen = False

        # mpi_run = False
        if self.mpi_run:
            iec.mpi_run = mpi_run
            iec.comm_map_down = mpi_comm_map_down

        case_inputs = {}
        case_inputs[("Fst", "TMax")] = {'vals': [TMax], 'group': 0}
        case_inputs[("Fst", "OutFileFmt")] = {'vals': [self.outfile_fmt], 'group': 0}

        case_inputs[('ServoDyn', 'GenTiStr')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ServoDyn', 'GenTiStp')] = {'vals': ['True'], 'group': 0}
        case_inputs[('ServoDyn', 'SpdGenOn')] = {'vals': [0.], 'group': 0}
        case_inputs[('ServoDyn', 'DLL_FileName')] = {'vals': [self.rosco_path], 'group': 0}

        case_inputs[("AeroDyn15", "WakeMod")] = {'vals': [1], 'group': 0}
        case_inputs[("AeroDyn15", "AFAeroMod")] = {'vals': [2], 'group': 0}
        case_inputs[("AeroDyn15", "TwrPotent")] = {'vals': [0], 'group': 0}
        case_inputs[("AeroDyn15", "TwrShadow")] = {'vals': ['False'], 'group': 0}
        case_inputs[("AeroDyn15", "TwrAero")] = {'vals': ['False'], 'group': 0}
        case_inputs[("AeroDyn15", "SkewMod")] = {'vals': [1], 'group': 0}
        case_inputs[("AeroDyn15", "TipLoss")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "HubLoss")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "TanInd")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "AIDrag")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "TIDrag")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn15", "IndToler")] = {'vals': [1.e-5], 'group': 0}
        case_inputs[("AeroDyn15", "MaxIter")] = {'vals': [5000], 'group': 0}
        case_inputs[("AeroDyn15", "UseBlCm")] = {'vals': ['True'], 'group': 0}

        if more_case_inputs:
            case_inputs.update(more_case_inputs)

        case_list, case_name_list = iec.execute(case_inputs=case_inputs)

        # Ensure proper output channels
        var_out = self.var_out

        channels = {}
        for var in var_out:
            channels[var] = True

        # Set up FAST Sims, move setup up
        fastBatch = runFAST_pywrapper_batch()
        fastBatch.FAST_ver = self.FAST_ver
        fastBatch.FAST_exe = self.FAST_exe   # Path to executable
        fastBatch.FAST_runDirectory = self.runDir
        fastBatch.FAST_InputFile = self.FAST_InputFile  # FAST input file (ext=.fst)
        fastBatch.FAST_directory = self.FAST_directory   # Path to fst directory files
        fastBatch.debug_level = self.debug_level
        fastBatch.dev_branch = self.dev_branch

        fastBatch.case_list = case_list
        fastBatch.case_name_list = case_name_list
        fastBatch.channels = channels

        # Check if simulation has been run
        if self.outfile_fmt == 1:
            outfile_ext = '.out'
        elif self.outfile_fmt == 2:
            outfile_ext = '.outb'
        elif self.outfile_fmt == 3:
            outfile_ext = '.outb'
        else:
            print('Warning, outfile format may be invalid. Attempting to read binary outputs.')
            outfile_ext = 'outb'
        outFileNames = [os.path.join(fastBatch.FAST_runDirectory, case_name + outfile_ext)
                        for case_name in case_name_list]
        outFileThere = [os.path.exists(outFileName) for outFileName in outFileNames]

        # Run simulations if they're not all there or if you want to overwrite
        if not all(outFileThere) or self.overwrite:
            if self.cores > 1:
                fastBatch.run_multi(self.cores)
            else:
                fastBatch.run_serial()

    def ROSCO_Controller_Comp(self, controller_paths, testtype='light'):
        '''
        Heavy or light testing for n controllers, n = len(controller_paths)

        Parameters:
        ----------
        controller_paths: list
            list of paths to .dlls
        testtype: str
            type of test to run. 'light' or 'heavy
        '''
        # Save initial run directory
        run_dir_init = self.runDir
        wind_dir_init = self.windDir
        for ci, path in enumerate(controller_paths):
            # specify rosco path
            self.rosco_path = path
            # temporarily change run directories
            self.runDir = os.path.join(run_dir_init,'controller_{}'.format(ci)) # specific directory for each controller
            self.windDir = os.path.join(run_dir_init, 'wind')  # wind in base runDir

            if testtype.lower() == 'light':
                self.ROSCO_Test_lite()
            elif testtype.lower() == 'heavy':
                self.ROSCO_Test_heavy()
            else:
                raise ValueError('{} is an invalid testtype for controller comparison'.format(testtype))

        # reset self
        self.runDir = run_dir_init
        self.windDir = wind_dir_init
    
    def ROSCO_DISCON_Comp(self, DISCON_filenames, testtype='light'):
        '''
        Heavy or light testing for n DISCON.IN files, n = len(DISCON_paths)

        Parameters:
        ----------
        controller_paths: list
            list of paths to .dlls
        testtype: str
            type of test to run. 'light' or 'heavy
        '''

        # Save initial run directory
        run_dir_init = self.runDir
        wind_dir_init = self.windDir
        for ci, discon in enumerate(DISCON_filenames):
            # temporarily change run directories
            self.runDir = os.path.join(run_dir_init, 'controller_{}'.format(ci))
            self.windDir = os.path.join(run_dir_init, 'wind')  # wind in base runDir

            # Point to different DISCON.IN files using more_case_inputs
            more_case_inputs = {}
            more_case_inputs[('ServoDyn', 'DLL_ProcName')] = {'vals': [discon], 'group': 0}
            self.windDir = os.path.join(run_dir_init, 'wind')  # wind in base runDir

            if testtype.lower() == 'light':
                self.ROSCO_Test_lite(more_case_inputs=more_case_inputs)
            elif testtype.lower() == 'heavy':
                self.ROSCO_Test_heavy(more_case_inputs=more_case_inputs)
            else:
                raise ValueError('{} is an invalid testtype for DISCON comparison'.format(testtype))

        # reset self
        self.runDir = run_dir_init
        self.windDir = wind_dir_init
