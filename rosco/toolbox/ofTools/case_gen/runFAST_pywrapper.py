"""
A basic python script that demonstrates how to use the FST8 reader, writer, and wrapper in a purely
python setting. These functions are constructed to provide a simple interface for controlling FAST
programmatically with minimal additional dependencies.
"""
# Hacky way of doing relative imports
from __future__ import print_function
import os, platform
import multiprocessing as mp

from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
from rosco.toolbox.ofTools.fast_io.FAST_writer import InputWriter_OpenFAST
from rosco.toolbox.ofTools.fast_io.FAST_wrapper import FAST_wrapper

# TODO: import weis and use library, import pCrunch and re-enable post-processing features available here

import numpy as np

mactype = platform.system().lower()
if mactype in ["linux", "linux2"]:
    libext = ".so"
elif mactype in ["win32", "windows", "cygwin"]: #NOTE: platform.system()='Windows', sys.platform='win32'
    libext = '.dll'
elif mactype == "darwin":
    libext = '.dylib'
else:
    raise ValueError('Unknown platform type: '+mactype)


# magnitude_channels_default = {
#     'LSShftF': ["RotThrust", "LSShftFys", "LSShftFzs"], 
#     'LSShftM': ["RotTorq", "LSSTipMys", "LSSTipMzs"],
#     'RootMc1': ["RootMxc1", "RootMyc1", "RootMzc1"],
#     'RootMc2': ["RootMxc2", "RootMyc2", "RootMzc2"],
#     'RootMc3': ["RootMxc3", "RootMyc3", "RootMzc3"],
#     'TipDc1': ['TipDxc1', 'TipDyc1', 'TipDzc1'],
#     'TipDc2': ['TipDxc2', 'TipDyc2', 'TipDzc2'],
#     'TipDc3': ['TipDxc3', 'TipDyc3', 'TipDzc3'],
#     'TwrBsM': ['TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt'],
# }

# fatigue_channels_default = {
#     'RootMc1': FatigueParams(slope=10),
#     'RootMc2': FatigueParams(slope=10),
#     'RootMc3': FatigueParams(slope=10),
#     'RootMyb1': FatigueParams(slope=10),
#     'RootMyb2': FatigueParams(slope=10),
#     'RootMyb3': FatigueParams(slope=10),
#     'TwrBsM': FatigueParams(slope=4),
#     'LSShftM': FatigueParams(slope=4),
# }
class runFAST_pywrapper(object):

    def __init__(self, **kwargs):

        self.FAST_exe           = None
        self.FAST_lib           = None
        self.FAST_InputFile     = None
        self.FAST_directory     = None
        self.FAST_runDirectory  = None
        self.FAST_namingOut     = None
        self.read_yaml          = False
        self.write_yaml         = False
        self.fst_vt             = {}
        self.case               = {}     # dictionary of variable values to change
        self.channels           = {}     # dictionary of output channels to change
        self.keep_time          = False
        self.use_exe            = True  # use openfast executable instead of library, helpful for debugging sometimes
        self.goodman            = False
        # self.magnitude_channels = magnitude_channels_default
        # self.fatigue_channels   = fatigue_channels_default
        self.la                 = None # Will be initialized on first run through
        self.allow_fails        = False
        self.fail_value         = 9999
        
        self.overwrite_outfiles = True   # True: existing output files will be overwritten, False: if output file with the same name already exists, OpenFAST WILL NOT RUN; This is primarily included for code debugging with OpenFAST in the loop or for specific Optimization Workflows where OpenFAST is to be run periodically instead of for every objective function anaylsis

        # Optional population class attributes from key word arguments
        for (k, w) in kwargs.items():
            try:
                setattr(self, k, w)
            except:
                pass

        super(runFAST_pywrapper, self).__init__()

    # def init_crunch(self):
    #     if self.la is None:
    #         self.la = LoadsAnalysis(
    #             outputs=[],
    #             magnitude_channels=self.magnitude_channels,
    #             fatigue_channels=self.fatigue_channels,
    #             #extreme_channels=channel_extremes_default,
    #         )
        
    def execute(self):

        # FAST version specific initialization
        reader = InputReader_OpenFAST()
        writer = InputWriter_OpenFAST()

        # Read input model, FAST files or Yaml
        if self.fst_vt == {}:
            reader.FAST_InputFile = self.FAST_InputFile
            reader.FAST_directory = self.FAST_directory
            reader.execute()
        
            # Initialize writer variables with input model
            writer.fst_vt = self.fst_vt = reader.fst_vt
        else:
            writer.fst_vt = self.fst_vt
        writer.FAST_runDirectory = self.FAST_runDirectory
        writer.FAST_namingOut = self.FAST_namingOut
        # Make any case specific variable changes
        if self.case:
            writer.update(fst_update=self.case)
        # Modify any specified output channels
        if self.channels:
            writer.update_outlist(self.channels)
        # Write out FAST model
        writer.execute()
        if self.write_yaml:
            writer.FAST_yamlfile = self.FAST_yamlfile_out
            writer.write_yaml()

        # Make sure pCrunch is ready
        # self.init_crunch()
            
        if not self.use_exe: # Use library
            raise Exception('ROSCO ofTools does not support running OpenFAST from a library, need to import WEIS.')

            # FAST_directory = os.path.split(writer.FAST_InputFileOut)[0]
            
            # orig_dir = os.getcwd()
            # os.chdir(FAST_directory)
        
            # openfastlib = FastLibAPI(self.FAST_lib, os.path.abspath(os.path.basename(writer.FAST_InputFileOut)))
            # openfastlib.fast_run()

            # output_dict = {}
            # for i, channel in enumerate(openfastlib.output_channel_names):
            #     output_dict[channel] = openfastlib.output_values[:,i]
            # del(openfastlib)
            
            # # Add channel to indicate failed run
            # output_dict['openfast_failed'] = np.zeros(len(output_dict[channel]))

            # output = OpenFASTOutput.from_dict(output_dict, self.FAST_namingOut, magnitude_channels=self.magnitude_channels)

            # # if save_file: write_fast
            # os.chdir(orig_dir)

            # if not self.keep_time: output_dict = None

        else: # use executable
            wrapper = FAST_wrapper()

            # Run FAST
            wrapper.FAST_exe = self.FAST_exe
            wrapper.FAST_InputFile = os.path.split(writer.FAST_InputFileOut)[1]
            wrapper.FAST_directory = os.path.split(writer.FAST_InputFileOut)[0]

            wrapper.allow_fails = self.allow_fails
            wrapper.fail_value  = self.fail_value

            FAST_Output     = os.path.join(wrapper.FAST_directory, wrapper.FAST_InputFile[:-3]+'outb')
            FAST_Output_txt = os.path.join(wrapper.FAST_directory, wrapper.FAST_InputFile[:-3]+'out')

            #check if OpenFAST is set not to overwrite existing output files, TODO: move this further up in the workflow for minor computation savings
            if self.overwrite_outfiles or (not self.overwrite_outfiles and not (os.path.exists(FAST_Output) or os.path.exists(FAST_Output_txt))):
                failed = wrapper.execute()
                if failed:
                    print('OpenFAST Failed! Please check the run logs.')
                    if self.allow_fails:
                        print(f'OpenFAST failures are allowed. All outputs set to {self.fail_value}')
                    else:
                        raise Exception('OpenFAST Failed! Please check the run logs.')
            else:
                failed = False
                print('OpenFAST not executed: Output file "%s" already exists. To overwrite this output file, set "overwrite_outfiles = True".'%FAST_Output)

            # if not failed:
            #     if os.path.exists(FAST_Output):
            #         output_init = OpenFASTBinary(FAST_Output, magnitude_channels=self.magnitude_channels)
            #     elif os.path.exists(FAST_Output_txt):
            #         output_init = OpenFASTAscii(FAST_Output, magnitude_channels=self.magnitude_channels)
                    
            #     output_init.read()

            #     # Make output dict
            #     output_dict = {}
            #     for i, channel in enumerate(output_init.channels):
            #         output_dict[channel] = output_init.df[channel].to_numpy()

            #     # Add channel to indicate failed run
            #     output_dict['openfast_failed'] = np.zeros(len(output_dict[channel]))

            #     # Re-make output
            #     output = OpenFASTOutput.from_dict(output_dict, self.FAST_namingOut)
            
            # else: # fill with -9999s
            #     output_dict = {}
            #     output_dict['Time'] = np.arange(self.fst_vt['Fst']['TStart'],self.fst_vt['Fst']['TMax'],self.fst_vt['Fst']['DT'])
            #     for module in self.fst_vt['outlist']:
            #         for channel in self.fst_vt['outlist'][module]:
            #             if self.fst_vt['outlist'][module][channel]:
            #                 output_dict[channel] = np.full(len(output_dict['Time']),fill_value=self.fail_value, dtype=np.uint8) 

            #     # Add channel to indicate failed run
            #     output_dict['openfast_failed'] = np.ones(len(output_dict['Time']), dtype=np.uint8)

            #     output = OpenFASTOutput.from_dict(output_dict, self.FAST_namingOut, magnitude_channels=self.magnitude_channels)



        # # Trim Data
        # if self.fst_vt['Fst']['TStart'] > 0.0:
        #     output.trim_data(tmin=self.fst_vt['Fst']['TStart'], tmax=self.fst_vt['Fst']['TMax'])
        # case_name, sum_stats, extremes, dels, damage = self.la._process_output(output,
        #                                                                        return_damage=True,
        #                                                                        goodman_correction=self.goodman)

        # return case_name, sum_stats, extremes, dels, damage, output_dict


class runFAST_pywrapper_batch(object):

    def __init__(self):

        run_dir                 = os.path.dirname( os.path.dirname( os.path.dirname( os.path.realpath(__file__) ) ) ) + os.sep
        self.FAST_exe           = os.path.join(run_dir, 'local/bin/openfast')   # Path to executable
        # self.FAST_lib           = os.path.join(lib_dir, 'libopenfastlib'+libext) 
        self.FAST_InputFile     = None
        self.FAST_directory     = None
        self.FAST_runDirectory  = None

        self.read_yaml          = False
        self.FAST_yamlfile_in   = ''
        self.fst_vt             = {}
        self.write_yaml         = False
        self.FAST_yamlfile_out  = ''

        self.case_list          = []
        self.case_name_list     = []
        self.channels           = {}

        self.overwrite_outfiles = True
        self.keep_time          = False

        self.goodman            = False
        # self.magnitude_channels = magnitude_channels_default
        # self.fatigue_channels   = fatigue_channels_default
        self.la                 = None
        self.use_exe            = True
        self.allow_fails        = False
        self.fail_value         = 9999
        
        self.post               = None

    # def init_crunch(self):
    #     if self.la is None:
    #         self.la = LoadsAnalysis(
    #             outputs=[],
    #             magnitude_channels=self.magnitude_channels,
    #             fatigue_channels=self.fatigue_channels,
    #             #extreme_channels=channel_extremes_default,
    #         )

    def create_case_data(self):

        case_data_all = []
        for i in range(len(self.case_list)):
            case_data = {}
            case_data['case']               = self.case_list[i]
            case_data['case_name']          = self.case_name_list[i]
            case_data['FAST_exe']           = self.FAST_exe
            # case_data['FAST_lib']           = self.FAST_lib
            case_data['FAST_runDirectory']  = self.FAST_runDirectory
            case_data['FAST_InputFile']     = self.FAST_InputFile
            case_data['FAST_directory']     = self.FAST_directory
            case_data['read_yaml']          = self.read_yaml
            case_data['FAST_yamlfile_in']   = self.FAST_yamlfile_in
            case_data['fst_vt']             = self.fst_vt
            case_data['write_yaml']         = self.write_yaml
            case_data['FAST_yamlfile_out']  = self.FAST_yamlfile_out
            case_data['channels']           = self.channels
            case_data['overwrite_outfiles'] = self.overwrite_outfiles
            case_data['use_exe']            = self.use_exe
            case_data['allow_fails']        = self.allow_fails
            case_data['fail_value']         = self.fail_value
            case_data['keep_time']          = self.keep_time
            case_data['goodman']            = self.goodman
            # case_data['magnitude_channels'] = self.magnitude_channels
            # case_data['fatigue_channels']   = self.fatigue_channels
            case_data['post']               = self.post

            case_data_all.append(case_data)

        return case_data_all
    
    def run_serial(self):
        # Run batch serially
        if not os.path.exists(self.FAST_runDirectory):
            os.makedirs(self.FAST_runDirectory)

        # self.init_crunch()
            
        case_data_all = self.create_case_data()
            
        # ss = {}
        # et = {}
        # dl = {}
        # dam = {}
        # ct = []
        for c in case_data_all:
            # _name, _ss, _et, _dl, _dam, _ct = evaluate(c)
            evaluate(c)
            # ss[_name] = _ss
            # et[_name] = _et
            # dl[_name] = _dl
            # dam[_name] = _dam
            # ct.append(_ct)
            
        # summary_stats, extreme_table, DELs, Damage = self.la.post_process(ss, et, dl, dam)

        # return summary_stats, extreme_table, DELs, Damage, ct

    def run_multi(self, cores=None):
        # Run cases in parallel, threaded with multiprocessing module

        if not os.path.exists(self.FAST_runDirectory):
            os.makedirs(self.FAST_runDirectory)

        if not cores:
            cores = mp.cpu_count()
        pool = mp.Pool(cores)

        # self.init_crunch()

        case_data_all = self.create_case_data()

        output = pool.map(evaluate_multi, case_data_all)
        pool.close()
        pool.join()

        # ss = {}
        # et = {}
        # dl = {}
        # dam = {}
        # ct = []
        # for _name, _ss, _et, _dl, _dam, _ct in output:
        #     ss[_name] = _ss
        #     et[_name] = _et
        #     dl[_name] = _dl
        #     dam[_name] = _dam
        #     ct.append(_ct)
            
        # summary_stats, extreme_table, DELs, Damage = self.la.post_process(ss, et, dl, dam)

        # return summary_stats, extreme_table, DELs, Damage, ct

    def run_mpi(self, mpi_comm_map_down):

        # Run in parallel with mpi
        from mpi4py import MPI

        # mpi comm management
        comm = MPI.COMM_WORLD
        rank = comm.Get_rank()
        sub_ranks = mpi_comm_map_down[rank]
        size = len(sub_ranks)

        N_cases = len(self.case_list)
        N_loops = int(np.ceil(float(N_cases)/float(size)))
        
        # file management
        if not os.path.exists(self.FAST_runDirectory) and rank == 0:
            os.makedirs(self.FAST_runDirectory)

        # self.init_crunch()

        case_data_all = self.create_case_data()

        output = []
        for i in range(N_loops):
            idx_s    = i*size
            idx_e    = min((i+1)*size, N_cases)

            for j, case_data in enumerate(case_data_all[idx_s:idx_e]):
                data   = [evaluate_multi, case_data]
                rank_j = sub_ranks[j]
                comm.send(data, dest=rank_j, tag=0)

            # for rank_j in sub_ranks:
            for j, case_data in enumerate(case_data_all[idx_s:idx_e]):
                rank_j = sub_ranks[j]
                data_out = comm.recv(source=rank_j, tag=1)
                output.append(data_out)

        # ss = {}
        # et = {}
        # dl = {}
        # dam = {}
        # ct = []
        # for _name, _ss, _et, _dl, _dam, _ct in output:
        #     ss[_name] = _ss
        #     et[_name] = _et
        #     dl[_name] = _dl
        #     dam[_name] = _dam
        #     ct.append(_ct)

        # summary_stats, extreme_table, DELs, Damage = self.la.post_process(ss, et, dl, dam)
        
        # return summary_stats, extreme_table, DELs, Damage, ct



def evaluate(indict):
    # Batch FAST pyWrapper call, as a function outside the runFAST_pywrapper_batch class for pickle-ablility

    # Could probably do this with vars(fast), but this gives tighter control
    known_keys = ['case', 'case_name', 'FAST_exe', 'FAST_lib', 'FAST_runDirectory',
                  'FAST_InputFile', 'FAST_directory', 'read_yaml', 'FAST_yamlfile_in', 'fst_vt',
                  'write_yaml', 'FAST_yamlfile_out', 'channels', 'overwrite_outfiles', 'keep_time',
                  'goodman','magnitude_channels','fatigue_channels','post','use_exe','allow_fails','fail_value']
    
    fast = runFAST_pywrapper()
    for k in indict:
        if k == 'case_name':
            fast.FAST_namingOut = indict['case_name']
        elif k in known_keys:
            setattr(fast, k, indict[k])
        else:
            print(f'WARNING: Unknown OpenFAST executation parameter, {k}')
    
    return fast.execute()

def evaluate_multi(indict):
    # helper function for running with multiprocessing.Pool.map
    # converts list of arguement values to arguments
    return evaluate(indict)
