'''
Update the DISCON.IN examples in the ROSCO repository using the Tune_Case/ .yaml files

'''
import os
from rosco.toolbox.ofTools.fast_io.update_discons import update_discons


if __name__=="__main__":

    # paths relative to Tune_Case/ and Test_Case/
    map_rel = {
        'NREL5MW.yaml': 'NREL-5MW/DISCON.IN',
        'IEA15MW.yaml': 'IEA-15-240-RWT-UMaineSemi/DISCON-UMaineSemi.IN',
        'BAR.yaml':     'BAR_10/BAR_10_DISCON.IN',
        'NREL2p8.yaml': 'NREL_2p8_127/NREL-2p8-127_DISCON.IN',
        'RM1_MHK.yaml': 'MHK_RM1/MHK_RM1_DISCON.IN',
    }

    # Directories
    test_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir = os.path.realpath(os.path.join(test_dir,'../Tune_Cases'))

    # Make paths absolute
    map_abs = {}
    for tune, test in map_rel.items():
        tune = os.path.join(tune_dir,tune)
        map_abs[tune] = os.path.join(test_dir,test)

    # Make discons    
    update_discons(map_abs)
