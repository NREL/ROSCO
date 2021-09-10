.. _api_change:

API changes between versions
============================

This page lists the main changes in the ROSCO API (input file) between different versions.

The changes are tabulated according to the line number, and flag name.
The line number corresponds to the resulting line number after all changes are implemented.
Thus, be sure to implement each in order so that subsequent line numbers are correct.


ROSCO v2.4.0 to ROSCO `develop`
-------------------------------
Two filter parameters were added to 
- change the high pass filter in the floating feedback module
- change the low pass filter of the wind speed estimator signal that is used in torque control

====== =================    ======================================================================================================================================================================================================
Added in ROSCO develop
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Flag Name           Example Value
====== =================    ======================================================================================================================================================================================================
27     F_WECornerFreq       0.20944             ! F_WECornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].
29     F_FlHighPassFreq     0.01000             ! F_FlHighPassFreq    - Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s].
====== =================    ======================================================================================================================================================================================================



ROSCO v2.4.0 to ROSCO v2.3.0
----------------------------
====== =================    ======================================================================================================================================================================================================
Added in ROSCO develop
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Flag Name           Example Value
====== =================    ======================================================================================================================================================================================================
27     F_WECornerFreq       0.20944             ! F_WECornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].
29     F_FlHighPassFreq     0.01000             ! F_FlHighPassFreq    - Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s].
====== =================    ======================================================================================================================================================================================================
