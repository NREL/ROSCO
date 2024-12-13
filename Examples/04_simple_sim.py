"""
04_simple_sim
-------------
Demonstrate the simple 1-DOF wind turbine simulator with ROSCO

In this example:

* Load turbine from saved pickle and tune a ROSCO controller
* Run and plot a step wind simulation using 1-DOF model in ``rosco.toolbox.sim`` and the ROSCO dynamic library

.. figure:: /source/figures/04_NREL5MW_SimpSim.png
   :align: center
   :width: 70%


Notes:

* You must have a compiled controller in ROSCO/rosco/lib/, and properly point to it using the `lib_name` variable.
* Using wind speed estimators in this simple simulation is known to cause problems. We suggest using WE_Mode = 0 in the DISCON.IN or increasing sampling rate of simulation as workarounds.
* The simple simulation is run twice to check that arrays are deallocated properly.

"""
# Python modules
import matplotlib.pyplot as plt 
import numpy as np
import os
# ROSCO toolbox modules 
from rosco import discon_lib_path as lib_name
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import sim as ROSCO_sim
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml

def main():
    # Load yaml file 
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    parameter_filename = os.path.join(tune_dir,'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Specify controller dynamic library path and name

    #directories
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # # Load turbine model from saved pickle
    turbine         = ROSCO_turbine.Turbine
    turbine         = turbine.load(os.path.join(example_out_dir,'01_NREL5MW_saved.p'))

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(tune_dir,path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory']),
        rot_source='txt',txt_filename=cp_filename
        )

    # Tune controller 
    controller      = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    # Write parameter input file
    param_filename = os.path.join(this_dir,'DISCON.IN')
    write_DISCON(
    turbine,controller,
    param_file=param_filename, 
    txt_filename=cp_filename
    )


    # Load controller library
    controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename,sim_name='sim1')

    # Load the simulator
    sim_1 = ROSCO_sim.Sim(turbine,controller_int)

    # Define a wind speed history
    dt = 0.025
    tlen = 1000      # length of time to simulate (s)
    ws0 = 7         # initial wind speed (m/s)
    t= np.arange(0,tlen,dt) 
    ws = np.ones_like(t) * ws0
    # add steps at every 100s
    for i in range(len(t)):
        ws[i] = ws[i] + t[i]//100

    # Run simulator and plot results
    sim_1.sim_ws_series(t,ws,rotor_rpm_init=4)

    # Load controller library again to see if we deallocated properly
    controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename,sim_name='sim_2')

    # Run simulator again and plot results
    sim_2 = ROSCO_sim.Sim(turbine,controller_int)
    sim_2.sim_ws_series(t,ws,rotor_rpm_init=4)

    # Check if simulations are equal
    np.testing.assert_almost_equal(sim_1.gen_speed,sim_2.gen_speed)

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir,'04_NREL5MW_SimpSim.png'), bbox_inches='tight')

if __name__ == "__main__":
    main()

