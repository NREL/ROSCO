"""
31_Startup
----------------
This example demonstrates turbine startup procedure.
The startup occurs in various stages depending on the provided input.
The states are as follows:

* Stage 1: Free-Wheeling of rotor
    
    In this stage the generator torque is 0 (set using ``PRC_R_Torque = 0``), the blade pitch angles are set to ``SU_FW_Pitch`` and the rotor is free to rotate.
    The rotor continues to free-wheel until the rotor speed exceeds ``SU_RotorSpeedThresh`` and has stayed in stage-1 for at least ``SU_FW_MinDuration`` seconds.
    If  both these criteria are met, then the startup procedure proceeds to the next stage.

* Stage 2 - [``SU_LoadStages_N`` + 2]: 
    In the next stages the startup procedure relinquishes the control of the blade pitch angles.
    The load on the rotor is increased in stages by changing ``PRC_R_Torque``.
    ROSCO reads an array of desired loads as ``SU_LoadStages``.
    It also reads ramp-duration and hold-duration for each load stage as ``SU_LoadRampDuration`` and ``SU_LoadHoldDuration`` respectively.
    At each stage, the startup procedure increases ``PRC_R_Torque`` from its previous value to its current value using a smooth ramp in the form of a sigma function.
    The duration of the ramp is read from the ``SU_LoadRampDuration`` array.
    Then, the ``PRC_R_Torque`` held constant at the current level for a period of time (``SU_LoadHoldDuration``).

The following figure demonstrate the startup procedure

.. image:: ../images/examples/32_startup.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing

rpm2RadSec = 2.0 * (np.pi) / 60.0
deg2rad = np.pi / 180.0

# directories
this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir, "examples_out")
os.makedirs(example_out_dir, exist_ok=True)

FULL_TEST = False


def main():
    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir, "Tune_Cases/IEA15MW.yaml")

    # Set DISCON input dynamically through yaml/dict
    controller_params = {}
    controller_params["DISCON"] = {}
    controller_params["DISCON"]["Echo"] = 1
    controller_params["LoggingLevel"] = 3
    controller_params["SU_Mode"] = 1
    controller_params["DISCON"]["SU_FW_Pitch"] = 45.0 * np.pi / 180.0
    controller_params["DISCON"]["SU_FW_MinDuration"] = 40.0
    controller_params["DISCON"]["SU_RotorSpeedThresh"] = 0.5 * (2 * np.pi) / 60.0

    controller_params["DISCON"]["SU_LoadStages_N"] = 2
    controller_params["DISCON"]["SU_LoadStages"] = [0.2, 1]
    controller_params["DISCON"]["SU_LoadRampDuration"] = [40, 40]
    controller_params["DISCON"]["SU_LoadHoldDuration"] = [40, 20]

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml = parameter_filename
    r.case_inputs = {}

    # Disable floating DOFs for clarity
    r.case_inputs[("ElastoDyn", "PtfmSgDOF")] = {"vals": ["False"], "group": 0}
    r.case_inputs[("ElastoDyn", "PtfmSwDOF")] = {"vals": ["False"], "group": 0}
    r.case_inputs[("ElastoDyn", "PtfmHvDOF")] = {"vals": ["False"], "group": 0}
    r.case_inputs[("ElastoDyn", "PtfmRDOF")] = {"vals": ["False"], "group": 0}
    r.case_inputs[("ElastoDyn", "PtfmPDOF")] = {"vals": ["False"], "group": 0}
    r.case_inputs[("ElastoDyn", "PtfmYDOF")] = {"vals": ["False"], "group": 0}

    run_dir = os.path.join(example_out_dir, "32_startup_demo/1_pitch")

    # Wind case
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [10],
        "TMax": 200,
    }
    if not FULL_TEST:
        r.wind_case_opts["TMax"] = 2

    r.case_inputs[("ElastoDyn", "BlPitch1")] = {"vals": [90], "group": 0}
    r.case_inputs[("ElastoDyn", "BlPitch2")] = {"vals": [90], "group": 0}
    r.case_inputs[("ElastoDyn", "BlPitch3")] = {"vals": [90], "group": 0}
    r.case_inputs[("ElastoDyn", "RotSpeed")] = {"vals": [0.0], "group": 0}

    # Run simulation
    os.makedirs(run_dir, exist_ok=True)
    r.controller_params = controller_params
    r.save_dir = run_dir
    r.run_FAST()

    # Plot output
    outfile = [os.path.join(run_dir, "IEA15MW_0.outb")]
    cases = {}
    cases["Baseline"] = ["Wind1VelX", "BldPitch1", "GenTq", "RotSpeed", "GenPwr"]
    fast_out = output_processing.output_processing()
    fastout = fast_out.load_fast_out(outfile)
    fast_out.plot_fast_out(cases=cases, showplot=False)

    plt.savefig(os.path.join(example_out_dir, "32_startup.png"))


if __name__ == "__main__":
    main()
