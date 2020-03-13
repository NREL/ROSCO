# History

---------------------
## 2.0.0 - (2020-03-04)
Admittedly poor versioning since the last release. Lots of updates...

### API Changes
* Re-org of some functionalities
    - turbine.load_from_text is now turbine.utilities 
    - utilities.write_param_file is now utilities.write_DISCON
* Include flap controller tuning methods and related inputs to DISCON.IN
* Remove unnecessary control inputs in DISCON.IN (`z_pitch_*`)

### Other changes
* Updates to floating filtering methods
* Updates to floating controller tuning methods to be more mathematically sound
* Generic flap tuning - employ reading of AeroDyn15 files with multiple distributed aerodynamic control inputs
* Test case updates and bug fixes
* Example updates to showcase all functionalities
* Updates to OpenFAST output processing and plotting scripts
* All related improvements and updates ROSCO controller itself
---------------------
## 1.0.1 - (2020-01-29)
* Major bug fixes in second order low-pass filters
* Minor bug fixes for pitch saturation and filter mode settings
* Minor updates in tuning scripts
---------------------
## 1.0.0 - (2020-01-22)
* Version 1.0 release - initial transition from DRC-Fortran with major updates and API changes


