# NREL's Reference OpenSource Controller (ROSCO) for wind turbine applications
NREL's Reference OpenSource Controller (ROSCO) for wind turbine applications uses the Bladed-style DISCON interface used by, e.g., OpenFAST, Bladed (versions 4.5 or earlier) and HAWC2.

## Introduction
The NREL Reference OpenSource Controller (ROSCO) provides an open, modular and fully adaptable baseline wind turbine controller to the scientific community. New control implementations can be added to the existing baseline controller, and in this way, convenient assessments of the proposed algorithms is possible. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code. ROSCO is being developed in Fortran and uses the Bladed-style DISCON controller interface. The compiled controller is configured by a single control settings parameter file, and can work with any wind turbine model and simulation software using the DISCON interface. Baseline parameter files are supplied for the NREL 5-MW and DTU 10-MW reference wind turbines.

## Compiling ROSCO
Compiling ROSCO to be used on your machine is made simple using [cmake](https://cmake.org/). 

1. Required Software to build ROSCO
* Fortran compiler (GNU compiler version above 4.6.0 or Intel compiler version above 11)
* C/C++ compiler
* GNU Make (version 3.81 or later)
* CMake (version 2.8.12 or later)

2.  Steps to compile
First, clone the git repository:
``` 
git clone https://github.com/nikhar-abbas/DRC_Fortran.git
```
Second, create a build directory and run cmake. From the ROSCO home directory:
```
mkdir build
cd build
cmake ..
make install
```

## Using ROSCO for Bladed
If you want to use the controller with DNV GL Bladed v4.5 or earlier (which still has support for the DISCON external controller interface), do the following:
1. Be sure to use and place the 32-bit DLL in the same folder as where you put your project .$PJ-file
2. Copy in that same folder the DISCON.IN controller configuration file
3. Set-up the 32-bit DLL as an external controller (Control -> Discrete External Controller -> Define...)
3. Open the DISCON.IN file with a text editor and copy its entire contents in the "External controller data:" section (Control -> Discrete External Controller -> Define...)
4. Run a "Power Production" simulation

## Referencing
When you use ROSCO in any publication, please cite the following paper:
* Mulders, S.P. and van Wingerden, J.W. "Delft Research Controller: an open-source and community-driven wind turbine baseline controller." Journal of Physics: Conference Series. Vol. 1037. No. 3. IOP Publishing, 2018. [Link to the paper](https://iopscience.iop.org/article/10.1088/1742-6596/1037/3/032009/meta)