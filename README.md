# NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications
NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications is a toolbox designed to ease controller implementation for the wind turbine researcher. Some primary capabilities include:
* Generic tuning of NREL's ROSCO controller
* Simple 1-DOF turbine simulations for quick controller capability verifications
* Parsing of OpenFAST input and output files


## Introduction
The NREL Reference OpenSource Controller (ROSCO) provides an open, modular and fully adaptable baseline wind turbine controller to the scientific community. The ROSCO toolbox leverages this architecture and implementation to provide a generic tuning process for the controller. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code for the controller and the toolbox. The ROSCO toolbox is a mostly-python code base with a number of functionalities.

* [ROSCO](https://github.com/NREL/ROSCO) - the fortran source code for the ROSCO controller. 
* [Examples](https://github.com/NREL/ROSCO_toolbox/tree/master/examples) - short working examples of the capabilities of the ROSCO toolbox. 
* [Tune_Cases](https://github.com/NREL/ROSCO_toolbox/tree/master/Tune_Cases) - example generic tuning scripts for a number of open-source reference turbines.
* [Test_Cases](https://github.com/NREL/ROSCO_toolbox/tree/master/Test_Cases) - numerous NREL 5MW bases cases to run for controller updates and comparisons. A "test-suite", if you will...
* [Matlab_Toolbox](https://github.com/NREL/ROSCO_toolbox/tree/master/Matlab_Toolbox) - MATLAB scripts to parse and plot simulation output data.
* [ofTools](https://github.com/NREL/ROSCO_toolbox/tree/master/ofTools) - A number of scripts to facilitate usage of OpenFAST and manage OpenFAST input and output files. 


## Documentation
All relevant documentation about the ROSCO toolbox and ROSCO controller can be found at through [ROSCO's readthedocs webpage](https://rosco-toolbox.readthedocs.io/en/latest/). Here, users can find the information on [installing the ROSCO toolbox](https://rosco-toolbox.readthedocs.io/en/latest/source/install.html#installing-the-rosco-toolbox) and [compiling ROSCO](https://rosco-toolbox.readthedocs.io/en/latest/source/install.html#compiling-rosco) for control purposes. Additionally, there is information on the standard workflow and uses cases for the ROSCO toolchain, and more. 

## Referencing
If the ROSCO Toolbox played a role in your research, please cite it. This software can be
cited as:

   NREL: ROSCO Toolbox. Version 2.2.0, https://github.com/NREL/rosco_toolbox, 2021.

For LaTeX users:

```
@misc{ROSCO_toolbox_2021,
    author = {NREL},
    title = {{ROSCO Toolbox. Version 2.2.0}},
    year = {2021},
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