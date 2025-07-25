""" 
19_update_discon_version
------------------------
Test and demonstrate update_discon_version() function for converting an old ROSCO input
to the current version
"""

import os
import shutil
from rosco import discon_lib_path
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.tune import update_discon_version

def main():
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)

    old_discon_filename = os.path.join(this_dir,'example_inputs','DISCON_v2.2.0.IN')        # An IEA-15MW input

    # Tuning yaml can be anything, does not have to correspond to old discon
    tuning_yaml = os.path.join(this_dir,'Tune_Cases','NREL5MW.yaml')        # dummy for now
    new_discon = os.path.join(this_dir,'examples_out','18_UPDATED_DISCON.IN')
    update_discon_version(
        old_discon_filename,
        tuning_yaml,
        new_discon
        )
    
    # Try using updated DISCON

    shutil.copyfile(
        os.path.join(this_dir,'Test_Cases', 'IEA-15-240-RWT', 'IEA-15-240-RWT', 'Cp_Ct_Cq.IEA15MW.txt'),
        os.path.join(this_dir,'examples_out','Cp_Ct_Cq.IEA15MW.txt')
    )  # Copy Cp table for testing 

    controller_int = ROSCO_ci.ControllerInterface(
        discon_lib_path,
        param_filename=new_discon,
        sim_name='sim1')
    controller_int.kill_discon()

    assert(controller_int.aviFAIL.value == 0)

if __name__ == "__main__":
    main()