# NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications
NREL's Reference OpenSource Controller (ROSCO) for wind turbine applications is a toolset designed to ease controller use and implementation for the wind turbine researcher. Some primary capabilities include:
* A reference controller with industry-standard functionality 
* Generic tuning of NREL's ROSCO controller
* Simple 1-DOF turbine simulations for quick controller capability verifications
* Parsing of OpenFAST input and output files


## Introduction
The NREL Reference OpenSource Controller (ROSCO) provides an open, modular and fully adaptable baseline wind turbine controller to the scientific community. The ROSCO toolbox leverages this architecture and implementation to provide a generic tuning process for the controller. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code for the controller and the toolbox. The ROSCO controller is implemented in FORTRAN, while the remainder of the toolset is a mostly-python code base with a number of functionalities.

* [ROSCO](https://github.com/NREL/ROSCO/tree/main/ROSCO) - the fortran source code for the ROSCO controller. 
* [Examples](https://github.com/NREL/ROSCO/tree/main/Examples) - short working examples of the capabilities of the ROSCO toolbox. 
* [Tune_Cases](https://github.com/NREL/ROSCO/tree/main/Tune_Cases) - example generic tuning scripts for a number of open-source reference turbines.
* [Test_Cases](https://github.com/NREL/ROSCO/tree/main/Test_Cases) - numerous NREL 5MW bases cases to run for controller updates and comparisons. A "test-suite", if you will...
* [Matlab_Toolbox](https://github.com/NREL/ROSCO/tree/main/Matlab_Toolbox) - MATLAB scripts to parse and plot simulation output data.
* [ofTools](https://github.com/NREL/ROSCO/tree/main/ROSCO_toolbox/ofTools) - A number of scripts to facilitate usage of OpenFAST and manage OpenFAST input and output files. 
* [linear](https://github.com/NREL/ROSCO/tree/main/ROSCO_toolbox/linear) - Scripts to aid with the use of linear models for controller tuning and simplified simulation. 


## Documentation
All relevant documentation about the ROSCO toolbox and ROSCO controller can be found at through [ROSCO's readthedocs webpage](https://rosco.readthedocs.io/en/latest/). Here, users can find the information on [installing the ROSCO tools](https://rosco.readthedocs.io/en/latest/source/install.html) for control purposes. Additionally, there is information on the [standard workflow](https://rosco.readthedocs.io/en/latest/source/standard_use.html), details of the input files, use cases for the ROSCO tool-chain, and more. 

## Issues and Discussion
If you find issues with any of the code that resides in this repository, it is encouraged for you to open a [GitHub issue](https://github.com/NREL/ROSCO/issues). If you have general questions or comments regarding the code, please start a [discussion via GitHub](https://github.com/NREL/ROSCO/discussions). We encourage you to use these resources for all ROSCO-related questions and comments, rather than other resources such as the FAST forums. This helps us keep ROSCO-related items centralized, and provides a singular place for the community to look when they have questions that might arise. Please keep in mind that we will do our very best to respond in a timely manner, but may take a few days to get back to you if you catch us during a busy time. 

## Contributing
If it wasn't obvious from _open-source_ being in the title of the tool-set, this is an open-source code base that we would love for the community to contribute to. If you find yourself fixing any bugs, writing new routines, or even making small typo changes, please submit a [pull request](https://github.com/NREL/ROSCO/pulls). 

## Survey
Please help us better understand the ROSCO user-base and how we can improve rosco through this brief survey:
[ROSCO toolchain survey](https://forms.office.com/Pages/ResponsePage.aspx?id=fp3yoM0oVE-EQniFrufAgGWnC45k8q5Kl90RBkHijqBUN0JTNzBJT1QwMjIzNDhCWDlDTUZPWDdMWC4u)

## Referencing
To reference the ROSCO source code directly, please use the following DOI:
[![DOI](https://zenodo.org/badge/220498357.svg)](https://zenodo.org/badge/latestdoi/220498357)

If the ROSCO Toolbox played a role in your research, please cite it. This software can be
cited as:

   NREL: ROSCO. Version 2.4.1, https://github.com/NREL/ROSCO, 2021.

For LaTeX users:

```
@misc{ROSCO_toolbox_2021,
    author = {NREL},
    title = {{ROSCO. Version 2.4.1}},
    year = {2021},
    publisher = {GitHub},
    journal = {GitHub repository},
    url = {https://github.com/NREL/ROSCO}
    }
```
If the ROSCO generic tuning theory and implementation played a roll in your research, please cite the following paper
```
@Article{wes-2021-19,
AUTHOR = {Abbas, N. and Zalkind, D. and Pao, L. and Wright, A.},
TITLE = {A Reference Open-Source Controller for Fixed and Floating Offshore Wind Turbines},
JOURNAL = {Wind Energy Science Discussions},
VOLUME = {2021},
YEAR = {2021},
PAGES = {1--33},
URL = {https://wes.copernicus.org/preprints/wes-2021-19/},
DOI = {10.5194/wes-2021-19}
}
```

## Additional Contributors and Acknowledgments
Primary contributions to ROSCO have been provided by researchers the National Renewable Energy Laboratory and the University of Colorado Boulder. Additionally, the ROSCO controller was built upon the foundations of the [Delft Research Controller](https://github.com/TUDelft-DataDrivenControl/DRC_Fortran). Much of the intellect behind these contributions has been inspired or derived from an extensive amount of work in the literature. The bulk of this has been cited through the primary publications about this work. 
