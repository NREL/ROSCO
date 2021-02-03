from __future__ import print_function
from ROSCO_toolbox.ofTools.fast_io.output_processing import output_processing
import ROSCO_toolbox

def FAST_IO_timeseries(fname):
    # interface to FAST_IO data load
    try:
        test = ROSCO_toolbox.__file__
    except:
        print('WARNING: ROSCO_toolbox required for wisdem.aeroelasticse.FAST_post.FAST_IO_timeseries')
    
    fast_out = output_processing.output_processing()
    fast_data = fast_out.load_fast_out(fname, verbose=True)[0]
    return fast_data
