"""
15_pass_through
---------------
Use the runFAST scripts to set up an example, use pass through in yaml
In this example:

* use run_FAST_ROSCO class to set up a test case
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir         = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)
    
    # Simulation config    
    r = run_FAST_ROSCO()

    parameter_filename = os.path.join(this_dir,'Tune_Cases/NREL5MW_PassThrough.yaml')
    run_dir = os.path.join(example_out_dir,'15_PassThrough')
    os.makedirs(run_dir,exist_ok=True)

    # Step wind simulation
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step
    r.wind_case_opts    = {
        'U_start': [10],
        'U_end': [15],
        'wind_dir': run_dir
        }
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()


if __name__=="__main__":
    main()