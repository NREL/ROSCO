# NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications
NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications is a toolbox designed to ease controller implementation for the wind turbine researcher. Some primary capabilities include:
* Generic auto-tuning of NREL's ROSCO controller
* Simple 1-DOF turbine simulations for quick controller capability verifications
* Parsing of OpenFAST input and output files

Block diagrams of these capabilities can be seen in [architecture.png](architecture.png) (which is a bit outdated as of November 2019...).

## Introduction
The NREL Reference OpenSource Controller (ROSCO) provides an open, modular and fully adaptable baseline wind turbine controller to the scientific community. The ROSCO toolbox leverages this architecture and implementation to provide a generic tuning process for the controller. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code for the controller and the toolbox. The ROSCO toolbox is a mostly-python code base with a number of functionalities.

### ROSCO Toolbox Overview
* [DRC_Fortran](https://github.com/nikhar-abbas/DRC_Fortran/tree/master) - the python source code for most of the toolbox capabilities.
* [Examples](https://github.com/NREL/ROSCO_toolbox/tree/master/examples) - short working examples of the capabilities of the ROSCO toolbox. 
* [Tune_Cases](https://github.com/NREL/ROSCO_toolbox/tree/master/Tune_Cases) - example generic tuning scripts for a number of open-source reference turbines.
* [Test_Cases](https://github.com/NREL/ROSCO_toolbox/tree/master/Test_Cases) - numerous NREL 5MW bases cases to run for controller updates and comparisons. A "test-suite", if you will...
* [Matlab_Toolbox](https://github.com/NREL/ROSCO_toolbox/tree/master/Matlab_Toolbox) - MATLAB scripts to parse and plot simulation output data (tools will eventually be available in python as well).

## Using the ROSCO Toolbox
There is a short (but _hopefully_ sweet) installation and run process for basic controller tuning...

### Installing the ROSCO Toolbox
No matter what you desire to do, you will need to install the ROSCO toolbox
First, clone the git repository and initiate the submodule:
``` 
git clone https://github.com/nikhar-abbas/ROSCO_tooblox.git
git submodule init
git submodule update
```
Second, install it from the cloned home directory
```
pip install -e .
```
AeroelasticSE, a part of NREL's WISDEM software, is necessary to have installed as well. This is available at [https://github.com/WISDEM/AeroelasticSE](https://github.com/WISDEM/AeroelasticSE). This can be installed similarly.

### Running ROSCO with Generic Tuning
The [Tune_Cases](Tune_Cases) folder hosts examples on what needs to happen to write the input file to the ROSCO controller. You will also need to have the ROSCO controller compiled. 

#### Compiling ROSCO
The controller itself is installed as a submodule in the ROSCO toolbox. For further information on compiling and running ROSCO itself, especially if you are on a Windows machine, we point you to the [DRC_Fortran github page](https://github.com/nikhar-abbas/DRC_Fortran/tree/develop). For Unix systems, (or Unix shell's on Windows), cmake makes it easy to compile. In order to compile the controller, you should run the following commands from the [DRC_Fortran](DRC_Fortran) folder (note: this folder will be updated with the move of DRC_Fortran to ROSCO).
```
mkdir build
cd build
cmake ..
make
```
These commands will compile a binary titled `libdiscon.*` in the build folder, which is the binary necessary run the controller. This should only need to be compiled once. 

#### ROSCO Toolbox Generic Tuning
IF you would like to run the generic tuning process for ROSCO, examples are shown in the [Tune_Cases](Tune_Cases) folder. When you run your own version of [tune_NREL5MW.py](Tune_Cases/tune_NREL5MW.py), you will have two files that are necessary to run the controller. 
1. `DISCON.IN` - the input file to `libdiscon.*`. When running the controller, `DISCON.IN` must be in the directory that `libdiscon.*` is pointing to, and must be called `DISCON.IN`
2. `Cp_Cq_Ct.txt` (or similar) - The contains rotor performance tables that are necessary to run the wind speed estimators in ROSCO. This can live wherever you desire, just be sure to point to it properly in `DISCON.IN`.

### Updating ROSCO Toolbox
Simple git commands should update the toolbox and controller as development continues:
```
git pull
git submodule update 
```

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

## Additional Contributors and Acknowledgments
Primary contributions to this work have been provided by researchers the National Renewable Energy Laboratory and TU Delft. Much of the intellect behind these contributions has been inspired or derived from an extensive amount of work. The bulk of this has been cited through the primary publications about this work. 

There are also some specific acknowledgements we would like to communicate:
* The setpoint smoothing regime implemented through the ROSCO controller was contributed by sowento GmbH. 