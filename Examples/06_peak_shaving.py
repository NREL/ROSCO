"""
06_peak_shaving
----------------

This example demonstrates the minimum pitch schedule of the NREL 5MW RWT.
In the ROSCO controller, the minimum pitch is determined from a lookup table from the (estimated) wind speed to the limit.
The ROSCO DISCON controller only has PS_Mode of 0 (disable) or 1 (enabled).

When creating a DISCON using the ROSCO toolbox, a few more options are available.
For PS_Mode = 1, the minimum pitch limit is calculated to limit the maximum thrust to a fraction of the un-limited maximum thrust, computed from the rotor's Ct table.
The ``ps_percent`` input (to the tuning yaml) is the fraction of the allowed thrust to the maximum, un-limited thrust.
For example, if the maximum thrust is 1e5 N, and we want to limite the maximum thrust to 8e4 N, ``ps_percent`` = 0.8.
Note that "percent" is a bit of a misnomer.

For PS_Mode = 2, the minimum pitch limit is calculated to maximize the Cp surface in below rated; this usually results in a non-zero pitch at low wind speeds.  
For large rotors, sometimes a minimum rotor speed is enforced to avoid resonance between the rotor period and the tower natural frequency; this results in a non-constant TSR across the below-rated operating points.
In this case, different pitch angles will be required to maximize Cp.

For PS_Mode = 3, both the thrust limiting and the power maximizing tuning routines determine the minimum pitch limit.
This example demonstrates PS_Mode = 3 as follows, compared to the steady state blade pitch operating points of the turbine.
The ROSCO toolbox uses Cp and Ct tables to compute these inputs to the ROSCO controller.

.. figure:: ../images/examples/06_MinPitch.png
   :align: center
   :width: 70%

"""

# Figure copy: cp ../Examples/examples_out/06_MinPitch.png source/figures/

# Python modules
import matplotlib.pyplot as plt 
import os
# ROSCO toolbox modules 
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.inputs.validation import load_rosco_yaml

def main():
    this_dir = os.path.dirname(__file__)
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    example_out_dir = os.path.join(this_dir,'examples_out')
    if not os.path.isdir(example_out_dir):
        os.makedirs(example_out_dir)

    # Load yaml file 
    parameter_filename = os.path.join(tune_dir,'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Ensure minimum generator speed at 50 rpm (for example's sake), turn on peak shaving and cp-maximizing min pitch
    controller_params['vs_minspd'] = 50/97
    controller_params['PS_Mode'] = 3

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory']),
        rot_source='txt',txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
        )
    # Tune controller 
    controller.tune_controller(turbine)

    # Plot minimum pitch schedule
    fig, ax = plt.subplots(1,1)
    ax.plot(controller.v, controller.pitch_op,label='Steady State Operation')
    ax.plot(controller.v, controller.ps_min_bld_pitch, label='Minimum Pitch Schedule')
    ax.legend()
    ax.set_xlabel('Wind speed (m/s)')
    ax.set_ylabel('Blade pitch (rad)')

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir,'06_MinPitch.png'))

if __name__ == "__main__":
    main()
