.. toctree::

.. _standard_use:

Standard Uses
=================
TODO: This needs a lot of expansion

Tuning a Controller
--------------------

.. _generate_discon:
Generating DISCON.IN
--------------------
IF you would like to run the generic tuning process for ROSCO, examples are shown in the :code:`Tune_Cases` folder. When you run your own version of :code:`tune_ROSCO.py`, you will have two files that are necessary to run the controller. 
1. `DISCON.IN` (or similar) - the input file to `libdiscon.*`. When running the controller in OpenFAST, `DISCON.IN` must be appropriately pointed to by the `DLL_FileName` parameter in ServoDyn. 
2. `Cp_Cq_Ct.txt` (or similar) - This file contains rotor performance tables that are necessary to run the wind speed estimators in ROSCO. This can live wherever you desire, just be sure to point to it properly with the `PerfFileName` parameter in `DISCON.IN`.
