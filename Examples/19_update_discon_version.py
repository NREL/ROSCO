""" 
19_update_discon_version
------------------------
Test and demonstrate update_discon_version() function for converting an old ROSCO input
to the current version
"""

import os
from rosco.toolbox.tune import update_discon_version

def main():
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)

    old_discon_filename = os.path.join(this_dir,'example_inputs','DISCON_v2.2.0.IN')        # An IEA-15MW input

    # Tuning yaml can be anything, does not have to correspond to old discon
    tuning_yaml = os.path.join(this_dir,'Tune_Cases','NREL5MW.yaml')        # dummy for now
    update_discon_version(
        old_discon_filename,
        tuning_yaml,
        os.path.join(this_dir,'examples_out','18_UPDATED_DISCON.IN')
        )

if __name__ == "__main__":
    main()