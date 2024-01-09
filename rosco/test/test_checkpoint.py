'''
Regression testing for ROSCO

Tests:
    restart
'''    

import os
import platform
import unittest
import numpy as np

# Python Modules
from shutil import copyfile

# ROSCO toolbox modules
from rosco import discon_lib_path
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.ofTools.fast_io import output_processing 
from rosco.toolbox.ofTools.case_gen.CaseLibrary import set_channels
from rosco.toolbox.ofTools.case_gen.CaseGen_General import CaseGen_General
from rosco.toolbox.ofTools.fast_io.FAST_writer import InputReader_OpenFAST, InputWriter_OpenFAST
from rosco.toolbox.utilities import run_openfast


class RegressionTesting(unittest.TestCase):
    def test_restart(self):
        this_dir = os.path.dirname(os.path.abspath(__file__))
        rosco_dir = os.path.dirname( os.path.dirname(this_dir) )
        test_out_dir = os.path.join(this_dir, 'test_out')

        # Load yaml file (Open Loop Case)
        tune_directory = os.path.join(rosco_dir,'Examples','Tune_Cases')
        parameter_filename = os.path.join(tune_directory, 'IEA15MW.yaml')

        inps = load_rosco_yaml(parameter_filename)
        path_params = inps['path_params']
        turbine_params = inps['turbine_params']
        controller_params = inps['controller_params']

        case_inputs = {}
        case_inputs[('Fst', 'TMax')] = {'vals': [3.], 'group': 0}
        case_inputs[('ServoDyn', 'DLL_FileName')] = {'vals': [discon_lib_path], 'group': 0}
        case_inputs[('Fst', 'ChkptTime')] = {'vals': [1.], 'group': 1}
        case_inputs[('Fst', 'OutFileFmt')] = {'vals': [2], 'group': 1}
        case_inputs[('Fst', 'DT')] = {'vals': [0.025], 'group': 0}
        case_inputs[('DISCON_in', 'LoggingLevel')] = {'vals': [2], 'group': 1}
        case_inputs[('ElastoDyn', 'RotSpeed')] = {'vals': [7], 'group': 0}

        # Generate cases
        run_dir = os.path.join(test_out_dir, 'restart')
        if not os.path.exists(run_dir):
            os.makedirs(run_dir)

        case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase='iea15_restart')
        channels = set_channels()

        # write files
        reader = InputReader_OpenFAST()
        writer = InputWriter_OpenFAST()
        reader.FAST_InputFile = path_params['FAST_InputFile']
        reader.FAST_directory = os.path.realpath(os.path.join( tune_directory, path_params['FAST_directory']))

        reader.execute()
        writer.fst_vt = reader.fst_vt
        writer.FAST_runDirectory = test_out_dir
        for case, case_name in zip(case_list, case_name_list):
            writer.FAST_namingOut = case_name
            writer.update(fst_update=case)
            writer.update_outlist(channels)
            writer.execute()

        # Run first case
        fastcall = 'openfast'
        run_openfast(
            test_out_dir,
            fastcall=fastcall,
            fastfile=writer.FAST_InputFileOut,
            chdir=True
        )
        copyfile(os.path.join(test_out_dir, writer.FAST_InputFileOut[:-3]+'outb'),
                os.path.join(test_out_dir, 'orig.outb') )
        # copyfile(os.path.join(test_out_dir, writer.FAST_InputFileOut[:-3]+'RO.dbg'),
        #         os.path.join(test_out_dir, 'orig.dbg') )

        # run restart case
        run_openfast(
            test_out_dir,
            fastcall=fastcall,
            fastfile=writer.FAST_InputFileOut[:-3]+'40',
            chdir=True,
            restart=True
        )
        copyfile(os.path.join(test_out_dir, writer.FAST_InputFileOut[:-3]+'outb'),
            os.path.join(test_out_dir, 'restart.outb'))
        # copyfile(os.path.join(test_out_dir, writer.FAST_InputFileOut[:-3]+'RO.dbg'),
        #     os.path.join(test_out_dir, 'restart.dbg'))

        output_files = [os.path.join(test_out_dir, 'orig.outb'), 
                        os.path.join(test_out_dir, 'restart.outb')]
        op = output_processing.output_processing()
        fastout = op.load_fast_out(output_files, tmin=0)

        if False: # Plotting for debug
            import matplotlib.pyplot as plt
            cases = {}
            cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed', 'NacYaw','GenPwr']
            fig, ax = op.plot_fast_out(cases=cases, showplot=False)
            plt.show()

        print(fastout[1]['GenPwr'], fastout[0]['GenPwr'])
        self.check_relative_error(fastout[1]['GenPwr'], fastout[0]['GenPwr'], 1e-3)

    def check_relative_error(self, meas, real, tol):
        '''check relative error'''
        error = np.linalg.norm(meas - real) / np.linalg.norm(real)
        self.assertTrue(error<=tol)

if __name__ == "__main__":
    unittest.main()
