.. ROSCO_toolbox documentation master file, created by
   sphinx-quickstart on Mon Aug  3 22:46:52 2020.

ROSCO toolbox documentation
===========================
.. only:: html

   :Version: |release|
   :Date: |today|

NREL's Reference OpenSource Controller (ROSCO) toolbox for wind turbine applications is a toolbox designed to ease controller implementation for the wind turbine researcher. The purpose of these documents is to provide information for the use of the ROSCO related toolchain. 

Standard Use:
----------
For the standard use case in OpenFAST, ROSCO will need to be compiled. This is made possible via the instructions found in the :ref:`compiling` instructions. Once the controller is compiled, the turbine model should point to the compiled binary. In OpenFAST, this is ensured by changing the `DLL_FileName` parameter in the ServoDyn input file. 

Additionally, an input file is needed for the ROSCO controller. Though the controller only needs to be compiled once, each individual turbine and controller tuning requires an input file. This input file is generically dubbed "DISCON.IN''. In OpenFAST, the `DLL_InFile` parameter should be set to point to the desired input file. The ROSCO toolbox is used to automatically generate the input file. These instructions are provided in the instructions for :ref:`generate_discon`.

**Documentation Directory**
.. toctree::
   :maxdepth: 2

   compiling.rst
   generating.rst



.. Indices and tables
.. ==================
.. 
.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`
