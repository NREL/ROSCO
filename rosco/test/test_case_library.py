'''
Unit testing for ROSCO

Tests:
    initialization
'''    

import os
import unittest

# ROSCO toolbox modules
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.utilities import run_openfast


def run_turbsim():
    this_dir = os.path.dirname(os.path.abspath(__file__))

    # Define openfast output filenames
    wind_directory = os.path.join(this_dir,'..','..','Examples', 'Test_Cases', 'Wind')
    turbsim_infile = '90m_12mps_twr.inp'

    run_openfast(wind_directory, fastcall='turbsim',
                fastfile=os.path.join(wind_directory, turbsim_infile), chdir=False)

    return os.path.realpath(os.path.join(wind_directory, turbsim_infile.split('.')[0] + '.bts'))


class UnitTesting(unittest.TestCase):
    def test_case_library(self):

        r = run_FAST_ROSCO()


        r.tuning_yaml = 'NREL5MW.yaml'
        r.save_dir    = os.path.join(os.path.dirname(__file__),'outputs','case_testing')

        # Steady state wind (can do mulitple wind speeds)
        r.wind_case_fcn = cl.power_curve
        r.wind_case_opts = {
            'TMax': 1,
            'U': [8],
        }
        r.run_FAST()

        # Single step in single simulation
        r.wind_case_fcn = cl.simp_step
        r.wind_case_opts = {
            'TMax': 1,
            'TStep': 0.5,
            'U_start': [8],
            'U_end': [10],
        }
        r.run_FAST()

        # Multiple steps in single simulation
        r.wind_case_fcn = cl.steps
        r.wind_case_opts = {
            'TMax': 1,
            'tt': [0.1, 0.2, 0.3],
            'U': [8, 9, 10],
            'U_0': 7
        }
        r.run_FAST()

        # User hub-height wind file
        r.wind_case_fcn = cl.user_hh
        r.wind_case_opts = {
            'TMax': 1,
            'wind_filenames': [os.path.join(r.save_dir,'steps.hh')],
        }
        r.run_FAST()

        # User turbsim file
        ts_file = run_turbsim()
        r.wind_case_fcn = cl.turb_bts
        r.wind_case_opts = {
            'TMax': 1,
            'wind_filenames': [ts_file],
        }
        r.run_FAST()

        # Ramp wind
        r.wind_case_fcn = cl.ramp
        r.wind_case_opts = {
            'TMax': 1,
            't_start': 0.1,
            't_end': 0.2,
            'U_start': 8,
            'U_end': 10,
        }
        r.run_FAST()

        # Direction change
        r.wind_case_fcn = cl.direction_change
        r.wind_case_opts = {
            'TMax': 1,
            'U': [8],
            'dir_start': [0],
            'dir_end': [10],
            't_start': [0.1],
            't_end': [0.2],
        }
        r.run_FAST()




if __name__ == "__main__":
    unittest.main()
