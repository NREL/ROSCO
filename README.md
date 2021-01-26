# NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications
NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications is a toolbox designed to ease controller implementation for the wind turbine researcher. Some primary capabilities include:
* Generic tuning of NREL's ROSCO controller
* Simple 1-DOF turbine simulations for quick controller capability verifications
* Parsing of OpenFAST input and output files

Block diagrams of these capabilities can be seen in [architecture.png](architecture.png).

## Introduction
The NREL Reference OpenSource Controller (ROSCO) provides an open, modular and fully adaptable baseline wind turbine controller to the scientific community. The ROSCO toolbox leverages this architecture and implementation to provide a generic tuning process for the controller. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code for the controller and the toolbox. The ROSCO toolbox is a mostly-python code base with a number of functionalities.

* [ROSCO](https://github.com/NREL/ROSCO) - the fortran source code for the ROSCO controller. 
* [Examples](https://github.com/NREL/ROSCO_toolbox/tree/master/examples) - short working examples of the capabilities of the ROSCO toolbox. 
* [Tune_Cases](https://github.com/NREL/ROSCO_toolbox/tree/master/Tune_Cases) - example generic tuning scripts for a number of open-source reference turbines.
* [Test_Cases](https://github.com/NREL/ROSCO_toolbox/tree/master/Test_Cases) - numerous NREL 5MW bases cases to run for controller updates and comparisons. A "test-suite", if you will...
* [Matlab_Toolbox](https://github.com/NREL/ROSCO_toolbox/tree/master/Matlab_Toolbox) - MATLAB scripts to parse and plot simulation output data.
* [ofTools](https://github.com/NREL/ROSCO_toolbox/tree/master/ofTools) - A number of scripts to facilitate usage of OpenFAST and manage OpenFAST input and output files. 

## Using the ROSCO Toolbox
Here is a short (but _hopefully_ sweet) installation and run process for basic controller tuning...

### Installing the complete ROSCO Toolbox
In order to fully leverage the controller tuning capabilities, [WISDEM](https://github.com/WISDEM/WISDEM) is used. This is made available through installation via [Anaconda](https://www.anaconda.com/). If you do not already have Anaconda installed on your machine, it is recommended that you install it to install WISDEM. Alternatively, you can install wisdem following the ``for developer'' instructions on the WISDEM home page and skip to step 2 here.

If you do not have WISDEM or the ROSCO toolbox installed and would like to install WISDEM, the ROSCO toolbox, and compile the controller please do the following: open your terminal or command prompt, navigate to the folder of your choosing, and enter the below text into the command line. This code block is broken up in a piece-wise description in the following sections.
1.  #### Install WISDEM 
	```
	conda config --add channels conda-forge
	conda create -y --name rosco-env python=3.8
	conda activate rosco-env
	conda install -y wisdem
	```

2.	#### Clone and Install the ROSCO toolbox
	```
	git clone https://github.com/NREL/ROSCO_toolbox.git
	cd ROSCO_toolbox
	git submodule init
	git submodule update
	conda install compilers 					# (Mac/Linux only)
	conda install m2w64-toolchain libpython     # (Windows only)
	python setup.py install
	```

### Alternatively...
If you wish to write your own scripts to leverage the ROSCO toolbox tools, but do not necessarily need the source code or to run any of the examples, the ROSCO toolbox is available via PyPi:
```
pip install rosco_toolbox
```
Note that if you do choose to install the ROSCO Toolbox this way, you will not have the source code. Additionally, you will need to download WISDEM and the ROSCO controller separately if you wish to use any of the ROSCO toolbox functionalities that need those software packages. 

#### Compiling ROSCO
The controller itself is installed as a submodule in the ROSCO toolbox. For further information on compiling and running ROSCO itself, or to download the release binaries directly, we point you to the [ROSCO github page](https://github.com/NREL/ROSCO_toolbox.git). If you wish to re-compile the ROSOCO toolbox, cmake provides easy to compiling on Unix based systems. In order to compile the controller, you should run the following commands from the ROSCO_toolbox folder.
```
cd ROSCO
mkdir build
cd build
cmake ..
make
```
Those familiar with mingw on windows can also compile ROSCO similarly. 

These commands will compile a binary titled `libdiscon.*` in the build folder, which is the binary necessary run the controller. This should only need to be compiled once. The extension should be `.dll`, `.so`, or `.dylib`, depending on the user operating system. 

### Running ROSCO with Generic Tuning
The [Tune_Cases](Tune_Cases) folder hosts examples on what needs to happen to write the input file to the ROSCO controller. See below on some details for compiling ROSCO:

#### ROSCO Toolbox Generic Tuning
IF you would like to run the generic tuning process for ROSCO, examples are shown in the [Tune_Cases](Tune_Cases) folder. When you run your own version of [tune_ROSCO.py](Tune_Cases/tune_ROSCO.py), you will have two files that are necessary to run the controller. 
1. `DISCON.IN` (or similar) - the input file to `libdiscon.*`. When running the controller in OpenFAST, `DISCON.IN` must be appropriately pointed to by the `DLL_FileName` parameter in ServoDyn. 
2. `Cp_Cq_Ct.txt` (or similar) - This file contains rotor performance tables that are necessary to run the wind speed estimators in ROSCO. This can live wherever you desire, just be sure to point to it properly with the `PerfFileName` parameter in `DISCON.IN`.

### Updating ROSCO Toolbox
Simple git commands should update the toolbox and controller as development continues:
```
git pull
git submodule update 
```
and then recompile and reinstall as necessary...

## Referencing
If the ROSCO Toolbox played a role in your research, please cite it. This software can be
cited as:

   ROSCO. Version 1.0.0 (2020). Available at https://github.com/nrel/rosco_toolbox.

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
If the ROSCO generic tuning theory and implementation played a roll in your research, please cite the following paper
```
@inproceedings{Abbas_WindTech2019,
	doi = {10.1088/1742-6596/1452/1/012002},
	url = {https://doi.org/10.1088%2F1742-6596%2F1452%2F1%2F012002},
	year = 2020,
	month = {jan},
	publisher = {{IOP} Publishing},
	volume = {1452},
	pages = {012002},
	author = {Nikhar J. Abbas and Alan Wright and Lucy Pao},
	title = {An Update to the National Renewable Energy Laboratory Baseline Wind Turbine Controller},
	journal = {Journal of Physics: Conference Series}
}
```
Additionally, if you have extensively used the [ROSCO](https://github.com/NREL/ROSCO) controller or [WISDEM](https://github.com/wisdem/wisdem), please cite them accordingly. 


## Additional Contributors and Acknowledgments
Primary contributions to the ROSCO Toolbox have been provided by researchers the National Renewable Energy Laboratory (Nikhar J. Abbas, Daniel Zalkind, Alan Wright, and Paul Fleming) and the University of Colorado Boulder (Lucy Pao). Much of the intellect behind these contributions has been inspired or derived from an extensive amount of work in the literature. The bulk of this has been cited through the primary publications about this work. 

There have been a number of contributors to the logic of the ROSCO controller itself. Please see the [ROSCO github page](https://github.com/NREL/ROSCO) for more information on who these contributors have been. 