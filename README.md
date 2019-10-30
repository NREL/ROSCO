# NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications
NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications is a toolbox designed to ease controller implementation for the wind turbine researcher. Some primary capabilities include:
* Generic auto-tuning of NREL's ROSCO controller
* Simple 1-DOF turbine simulations for quick controller capability verifications
* Parsing of OpenFAST input and output files

Block diagrams of these capabilities can be seen in [architecture.png](architecture.png) (which is a bit outdated as of November 2019...).

## Introduction
The NREL Reference OpenSource Controller (ROSCO) provides an open, modular and fully adaptable baseline wind turbine controller to the scientific community. The ROSCO_toolbox leverages this architecture and implementation to provide a generic tuning process for the controller. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code for the controller and the toolbox. The ROSCO toolbox is a mostly-python code base with a number of functionalities.

### ROSCO Toolbox Overview
* [DRC_Fortran](DRC_Fortran) - the python source code for most of the toolbox capabilities.
* [Examples](Examples) - short working examples of the capabilities of the ROSCO toolbox. 
* [Tune_Cases](Tune_Cases) - example generic tuning scripts for a number of open-source reference turbines.
* [Test_Cases](Test_Cases) - numerous NREL 5MW bases cases to run for controller updates and comparisons. A "test-suite", if you will...
* [Matlab_Toolbox](Matlab_Toolbox) - MATLAB scripts to parse and plot simulation output data (tools will eventually be available in python as well).

## Installing the  ROSCO Toolbox
First, clone the git repository:
``` 
git clone https://github.com/nikhar-abbas/ROSCO_tooblox.git
```
Second, install it from the cloned home directory
```
pip install -e .
```
AeroelasticSE, a part of NREL's WISDEM software, is necessary to have installed as well. This is available at [https://github.com/WISDEM/AeroelasticSE](https://github.com/WISDEM/AeroelasticSE). This can be installed similarly.


## Referencing
If the ROSCO Toolbox played a role in your research, please cite it. This software can be
cited as:

   ROSCO. Version 0.1.0 (2019). Available at https://github.com/nrel/rosco_toolbox.

For LaTeX users:

```
    @misc{ROSCO_toolbox_2019,
    author = {NREL},
    title = {{ROSCO Toolbox. Version 0.1.0}},
    year = {2019},
    publisher = {GitHub},
    journal = {GitHub repository},
    url = {https://github.com/NREL/rosco_toolbox}
    }
```

~~ NJA - add NAWEA/WindTech paper here once published ~~
