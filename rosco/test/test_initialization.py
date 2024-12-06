'''
Unit testing for ROSCO

Tests:
    initialization
'''    

import os
import unittest


# ROSCO toolbox modules
from rosco import discon_lib_path
from rosco.toolbox import control_interface as ROSCO_ci


class UnitTesting(unittest.TestCase):
    def test_initialization(self):
        
        this_dir = os.path.dirname(os.path.abspath(__file__))
        param_filename = os.path.join(this_dir,'../../Examples/Test_Cases/IEA-15-240-RWT-UMaineSemi/DISCON-UMaineSemi.IN')


        # Load controller library
        ci_1 = ROSCO_ci.ControllerInterface(discon_lib_path,param_filename=param_filename,sim_name='sim1')

        ci_2 = ROSCO_ci.ControllerInterface(discon_lib_path,param_filename=param_filename,sim_name='sim2')

        # Check that the error is as expected
        assert("b'ROSCO:ERROR: This ROSCO dynamic library has already been loaded" == str(ci_2.avcMSG.raw).split('.')[0])


if __name__ == "__main__":
    unittest.main()
