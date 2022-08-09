import os
import unittest
from time import time
import runpy

all_scripts = [
    'example_01',
    'example_02',
    'example_03',
    'example_04',
    'example_05',
    'example_06',
    'example_07',
    'example_08',
    'example_09',
    'example_10',
    'example_11',
    'example_12',
    'example_13',
    'example_14',
    'example_15',
    'example_16',
    'example_17', # NJA: only runs on unix in CI
]

def execute_script(fscript):
    examples_dir = os.path.dirname(os.path.realpath(__file__))

    # Go to location due to relative path use for airfoil files
    print("\n\n")
    print("NOW RUNNING:", fscript)
    print()
    fullpath = os.path.join(examples_dir, fscript + ".py")
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
