1. May have to define objInst as a peresistent variable in function SetParameters.m
2. Figure out the nature of "inst" variable used in all the filter subroutines
3. LocalVar.PC_PitComT is not defined before it is called in line 44 of PreFilteredMeasuredSignal.m, therefore assigned a value equal to zero in SetParameters.m
4. Check the location of definition of persistent variables in WindSpeedEstimator.m 
5.  LocalVar.WE_Vw command added at line 74 of WindSpeedEstimator.m
6. The loops in WindSpeedEstimator.m are altered to pass LocalVar.WE_Vw_F as output on every call and at all control flag settings
7. PitComT_Last initialized in line23-25 PitchControl.m