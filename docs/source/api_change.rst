.. _api_change:

API changes between versions
============================

This page lists the main changes in the ROSCO API (input file) between different versions.

The changes are tabulated according to the line number, and flag name.
The line number corresponds to the resulting line number after all changes are implemented.
Thus, be sure to implement each in order so that subsequent line numbers are correct.


ROSCO v2.4.1 to ROSCO v2.5.0
-------------------------------
Two filter parameters were added to 
- change the high pass filter in the floating feedback module
- change the low pass filter of the wind speed estimator signal that is used in torque control

Open loop control inputs, users must specify:
- The open loop input filename, an example can be found in Examples/Example_OL_Input.dat
- Indices (columns) of values specified in OL_Filename

IPC
- Proportional Control capabilities were added, 1P and 2P gains should be specified

====== =================    ======================================================================================================================================================================================================
Added in ROSCO develop
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Flag Name           Example Value
====== =================    ======================================================================================================================================================================================================
27     F_WECornerFreq       0.20944             ! F_WECornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].
29     F_FlHighPassFreq     0.01000             ! F_FlHighPassFreq  - Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s].
50     IPC_KP               0.000000  0.000000  ! IPC_KP			- Proportional gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]
125    OL_Filename          "14_OL_Input.dat"   ! OL_Filename       - Input file with open loop timeseries (absolute path or relative to this file)
126    Ind_Breakpoint       1                   ! Ind_Breakpoint    - The column in OL_Filename that contains the breakpoint (time if OL_Mode = 1)
127    Ind_BldPitch         2                   ! Ind_BldPitch      - The column in OL_Filename that contains the blade pitch input in rad
128    Ind_GenTq            3                   ! Ind_GenTq         - The column in OL_Filename that contains the generator torque in Nm
129    Ind_YawRate          4                   ! Ind_YawRate       - The column in OL_Filename that contains the generator torque in Nm
====== =================    ======================================================================================================================================================================================================
