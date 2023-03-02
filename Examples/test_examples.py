import os
import unittest
from time import time
import runpy

all_scripts = [
    '01_turbine_model',
    '02_ccblade',
    '03_tune_controller',
    '04_simple_sim',
    '05_openfast_sim',
    '06_peak_shaving',
    '07_openfast_outputs',
    '08_run_turbsim',
    '09_distributed_aero',
    '10_linear_params',
    '11_robust_tuning',
    '12_tune_ipc',
    '14_open_loop_control',
    '15_pass_through',
    '16_external_dll',
    '17_zeromq_interface',  # NJA: only runs on unix in CI
    '18_pitch_offsets',
    '19_update_discon_version', 
    '21_optional_inputs',
    '22_cable_control',
    'update_rosco_discons',     
]

def execute_script(fscript):
    examples_dir = os.path.dirname(os.path.realpath(__file__))
    test_case_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)),'../Test_Cases'))

    # Go to location due to relative path use for airfoil files
    print("\n\n")
    print("NOW RUNNING:", fscript)
    print()

    if fscript in ['update_rosco_discons']:
        run_dir = test_case_dir
    else:
        run_dir = examples_dir

    fullpath = os.path.join(run_dir, fscript + ".py")
    basepath = os.path.dirname(os.path.realpath(fullpath))
    os.chdir(basepath)

    # Use runpy to execute examples
    s = time()
    runpy.run_path(os.path.realpath(fullpath), run_name='__main__')
    print(time() - s, "seconds to run")

class TestExamples(unittest.TestCase):

    def test_all_scripts(self):
        for ks, s in enumerate(all_scripts):
            with self.subTest(f"Running: {s}", i=ks):
                try:
                    execute_script(s)
                    self.assertTrue(True)
                except:
                    self.assertEqual(s, "Success")    


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestExamples))
    return suite


if __name__ == "__main__":
    result = unittest.TextTestRunner().run(suite())

    if result.wasSuccessful():
        exit(0)
    else:
        exit(1)
