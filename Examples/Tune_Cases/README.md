# Tuning cases
Here are some instructions on how to use the generic tuning capabilities of the ROSCO toolbox. 

## Basic Steps
The basic steps are as follows

1. Fill out the .yaml input file to tune the controller
2. Run `tune_ROSCO.py`, or your own version of it.
3. Copy `DISCON.IN` and `Cp_Ct_Cq.txt` (or similar) to the desired folder for your wind turbine model.
4. Ensure that `PerfFileName` in `DISCON.IN` points to the proper name and location of `Cp_Ct_Cq.*.txt`
5. Ensure that `DLL_InFile` in the OpenFAST ServoDyn input file properly points to `DISCON.IN`.
6. Enjoy the results. Possibly retune your controller. 

## The .yaml File
We use a .yaml file to define the inputs to the generic controlling scripts. This .yaml file defines three different dictionaries with a number of parameters. The parameters that must be defined in [the documentation](https://rosco.readthedocs.io/en/latest/source/rosco_toolbox.html#the-rosco-toolbox-tuning-file)


### The controller parameters
The controller flags have some default values for the ROSCO controller implementation (provided in the previous section). This, subsequently, leaves four turbine parameters that the user must decide `zeta_pc`, `omega_pc`, `zeta_vs`, and `omega_vs`. The example .yaml scripts provided offer some insight into what these values might be for different sizes and types of turbines. Generally speaking, we desire slower responses in the the variable speed (vs) torque controller. This equates to `zeta_vs = 1` and `omega_vs = 0.3` for the NREL 5MW wind turbine. These values tend to translate fairly well to larger turbines, but decreasing `omega_vs = 0.3` may be desired. For the pitch controller, the NREL 5MW turbine was tuned such that `zeta_pc = 0.7`, and `omega_pc = 0.6` in the legacy controller. As a high level rule of thumb, increasing the desired damping to be over-damped (`zeta_pc ~ 1.0`) and the natural frequency to be much slower (`omega_pc = 0.2`) seems to produce smoother wind turbine responses in large turbines with highly flexible rotors. 


## Running a tuning case
As mentioned above, a sample tuning script `tune_ROSCO.py` is provided. This script only needs the first line to be modified to point to the desired `*.yaml` file, and will produce the necessary input files for the controller. The curious user should feel free to modify and change this file. A simple addition is to include the use of `sim.py` to run a simple 1DOF simulation case to verify controller tuning - `example_06.py` shows some of this functionally. Other uses include running OpenFAST directly (see `example_08.py`), plotting system outputs (functionality pending), or visualizing results from ccblade rotor performance analysis (Cp surface is plotted currently).