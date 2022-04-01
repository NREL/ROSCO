ROSCO Documentation
===================
:Version: |release|
:Date: |today|

NREL's Reference OpenSource Controller (ROSCO) tool-set for wind turbine applications designed to ease controller implementation for the wind turbine researcher. The purpose of these documents is to provide information for the use of the tool-set. 

:numref:`fig-RT` shows the general workflow for the ROSCO tool-chain. with OpenFAST

.. _fig-RT:
.. figure:: /source/figures/ROSCO_toolbox.svg
   :align: center
   :width: 80%

   ROSCO toolchain general workflow


**ROSCO Toolbox**
The python-based toolbox primarily used for tuning the controller and writing the DISCON.IN. 

* Generic tuning of NREL's ROSCO controller
* Simple 1-DOF turbine simulations for quick controller capability verifications
* Parsing of OpenFAST input and output files
* Linear model analysis capability

**ROSCO Controller**
The controller implementation itself. This is compiled to :code:`libdiscon.*` file, reads the DISCON.IN file, and interfaces with OpenFAST using the Bladed-style interface. 

* Fortran based
* Follows Bladed-style control interface
* Modular

Standard Use
------------

For the standard use case in OpenFAST (or similar), ROSCO will need to be compiled. This is made possible via the instructions found in :ref:`install`. Once the controller is compiled, the turbine model needs to point to the compiled binary. In OpenFAST, this is ensured by changing the :code:`DLL_FileName` parameter in the ServoDyn input file. 

Additionally, an additional input file is needed for the ROSCO controller. Though the controller only needs to be compiled once, each individual turbine/controller tuning requires an input file. This input file is generically dubbed "DISCON.IN''. In OpenFAST, the :code:`DLL_InFile` parameter should be set to point to the desired input file. The ROSCO toolbox is used to automatically generate the input file. These instructions are provided in the instructions for :ref:`standard_use`.

Technical Documentation
-----------------------
A publication highlighting much of the theory behind the controller tuning and implementation methods can be found at:
https://wes.copernicus.org/preprints/wes-2021-19/

Survey
------
Please help us better understand the ROSCO user-base and how we can improve ROSCO through this brief survey:

.. raw:: html
   
   <iframe width="640px" height= "480px" src= "https://forms.office.com/Pages/ResponsePage.aspx?id=fp3yoM0oVE-EQniFrufAgGWnC45k8q5Kl90RBkHijqBUN0JTNzBJT1QwMjIzNDhCWDlDTUZPWDdMWC4u&embed=true" frameborder= "0" marginwidth= "0" marginheight= "0" style= "border: none; max-width:100%; max-height:100vh" allowfullscreen webkitallowfullscreen mozallowfullscreen msallowfullscreen> </iframe>
   


License
-------
Copyright 2021 NREL

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

.. toctree::
   :maxdepth: 3
   :numbered:
   
   source/install.rst
   source/standard_use.rst
   source/rosco_toolbox.rst
   source/rosco.rst
   source/api_change.rst
   source/toolbox_input.rst
   source/ROSCO_instructions_for_Bladed.rst
