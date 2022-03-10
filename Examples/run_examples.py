import os
import unittest
import sys
from time import time
import importlib

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
    'example_14'
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

        # Get script/module name
        froot = fscript.split("/")[-1]

        # Use dynamic import capabilities
        # https://www.blog.pythonlibrary.org/2016/05/27/python-201-an-intro-to-importlib/
        print(froot, os.path.realpath(fullpath))
        spec = importlib.util.spec_from_file_location(froot, os.path.realpath(fullpath))
        mod = importlib.util.module_from_spec(spec)
        s = time()
        spec.loader.exec_module(mod)
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
