.. _api_change:

API changes between versions
============================

This page lists the main changes in the ROSCO API (input file) between different versions.

The changes are tabulated according to the line number, and flag name.
The line number corresponds to the resulting line number after all changes are implemented.
Thus, be sure to implement each in order so that subsequent line numbers are correct.

2.8.0 to develop
-------------------------------
Gain scheduling of floating feedback
-  The floating feedback gain can be scheduled on the low pass filtered wind speed signal.  Note that Fl_Kp can now be an array.
-  Control all three blade pitch inputs in open loop
-  Rotor position control of azimuth position with PID (RP_Gains) control inputs
====== =================    ======================================================================================================================================================================================================
Changed in ROSCO develop
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
126     OL_mode             0           ! OL_Mode           - Open loop control mode {0: no open loop control, 1: open loop control vs. time, 2: rotor position control}
126     Fl_Kp               0.0000      ! Fl_Kp             - Nacelle velocity proportional feedback gain [s]
139     Ind_BldPitch        0   0   0   ! Ind_BldPitch      - The columns in OL_Filename that contains the blade pitch (1,2,3) inputs in rad [array]
====== =================    ======================================================================================================================================================================================================


====== =================    ======================================================================================================================================================================================================
New in ROSCO develop
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
125     Fl_n                1                       ! Fl_n          - Number of Fl_Kp gains in gain scheduling, optional with default of 1
127     Fl_U                0.0000                  ! Fl_U          - Wind speeds for scheduling Fl_Kp, optional if Fl_Kp is single value [m/s]
142     Ind_Azimuth         0                       ! Ind_Azimuth   - The column in OL_Filename that contains the desired azimuth position in rad (used if OL_Mode = 2)
143     RP_Gains            0.0000 0.0000 0.0000    ! RP_Gains      - PID gains for rotor position control (used if OL_Mode = 2)
====== =================    ======================================================================================================================================================================================================



2.7.0 to 2.8.0
-------------------------------
Optional Inputs
-  ROSCO now reads in the whole input file and searches for keywords to set the inputs.  Blank spaces and specific ordering are no longer required.
-  Input requirements depend on control modes.  E.g., open loop inputs are not required if `OL_Mode = 0``
Cable Control
-  Can control OpenFAST cables (MoorDyn or SubDyn) using ROSCO
Structural Control
-  Can control OpenFAST structural control elements (ServoDyn) using ROSCO
Active wake control
-  Added Active Wake Control (AWC) implementation

====== =================    ======================================================================================================================================================================================================
New in ROSCO 2.8.0
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
6      Echo                 0               ! Echo		    - (0 - no Echo, 1 - Echo input data to <RootName>.echo)
25     AWC_Mode             0			    ! AWC_Mode       - Active wake control mode [0 - not used, 1 - complex number method, 2 - Coleman transform method]
28     CC_Mode              0               ! CC_Mode           - Cable control mode [0- unused, 1- User defined, 2- Open loop control]
29     StC_Mode             0               ! StC_Mode          - Structural control mode [0- unused, 1- User defined, 2- Open loop control]
139    Ind_CableControl     0               ! Ind_CableControl  - The column(s) in OL_Filename that contains the cable control inputs in m [Used with CC_Mode = 2, must be the same size as CC_Group_N]
140    Ind_StructControl    0               ! Ind_StructControl - The column(s) in OL_Filename that contains the structural control inputs [Used with StC_Mode = 2, must be the same size as StC_Group_N]
148    Empty Line
149    AWC_Section          !------- Active Wake Control -----------------------------------------------------
150    AWC_NumModes         1               ! AWC_NumModes    - AWC- Number of modes to include [-]
151    AWC_n                1               ! AWC_n           - AWC azimuthal mode [-] (only used in complex number method)
152    AWC_harmonic         1               ! AWC_harmonic    - AWC Coleman transform harmonic [-] (only used in Coleman transform method)
153    AWC_freq             0.03            ! AWC_freq        - AWC frequency [Hz]
154    AWC_amp              2.0             ! AWC_amp         - AWC amplitude [deg]
155    AWC_clockangle       0.0             ! AWC_clockangle  - AWC clock angle [deg]
165    Empty Line          
166    CC_Section           !------- Cable Control ---------------------------------------------------------
167    CC_Group_N           3               ! CC_Group_N		- Number of cable control groups
168    CC_GroupIndex        2601 2603 2605  ! CC_GroupIndex  - First index for cable control group, should correspond to deltaL
169    CC_ActTau            20.000000       ! CC_ActTau		- Time constant for line actuator [s]
170    Empty Line          
171    StC_Section          !------- Structural Controllers ---------------------------------------------------------
172    StC_Group_N          3               ! StC_Group_N		- Number of cable control groups
173    StC_GroupIndex       2818 2838 2858  ! StC_GroupIndex     - First index for structural control group, options specified in ServoDyn summary output   
====== =================    ======================================================================================================================================================================================================


2.6.0 to 2.7.0
-------------------------------
Pitch Faults
-  Constant pitch actuator offsets (PF_Mode = 1)
IPC Saturation Modes
-  Added options for saturating the IPC command with the peak shaving limit

====== =================    ======================================================================================================================================================================================================
New in ROSCO 2.7.0
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
23     PF_Mode              0                   ! PF_Mode           - Pitch fault mode {0 - not used, 1 - constant offset on one or more blades}
56     IPC_SatMode          2                   ! IPC_SatMode		- IPC Saturation method (0 - no saturation (except by PC_MinPit), 1 - saturate by PS_BldPitchMin, 2 - saturate sotfly (full IPC cycle) by PC_MinPit, 3 - saturate softly by PS_BldPitchMin)
139    PF_Section           !------- Pitch Actuator Faults ---------------------------------------------------------
140    PF_Offsets           0.00000000 0.00000000 0.00000000                 ! PF_Offsets     - Constant blade pitch offsets for blades 1-3 [rad]
141    Empty Line          
====== =================    ======================================================================================================================================================================================================


2.5.0 to 2.6.0
-------------------------------
IPC
-  A wind speed based soft cut-in using a sigma interpolation is added for the IPC controller

Pitch Actuator
-  A first or second order filter can be used to model a pitch actuator

External Control Interface
-  Call another control library from ROSCO

ZeroMQ Interface
-  Communicate with an external routine via ZeroMQ. Only yaw control currently supported

Updated yaw control
-  Filter wind direction with deadband, and yaw until direction error changes signs (https://iopscience.iop.org/article/10.1088/1742-6596/1037/3/032011)

====== =================    ======================================================================================================================================================================================================
New in ROSCO 2.6.0
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
19     TD_Mode              0                    ! TD_Mode           - Tower damper mode {0: no tower damper, 1: feed back translational nacelle accelleration to pitch angle}
22     PA_Mode              0                    ! PA_Mode           - Pitch actuator mode {0 - not used, 1 - first order filter, 2 - second order filter}
23     Ext_Mode             0                    ! Ext_Mode          - External control mode {0 - not used, 1 - call external dynamic library}
24     ZMQ_Mode             0                    ! ZMQ_Mode          - Fuse ZeroMQ interaface {0: unused, 1: Yaw Control}
33     F_YawErr             0.17952              ! F_YawErr          - Low pass filter corner frequency for yaw controller [rad/s].
54     IPC_Vramp            9.120000  11.400000  ! IPC_Vramp	     - Start and end wind speeds for cut-in ramp function. First entry: IPC inactive, second entry: IPC fully active. [m/s]
96     Y_uSwitch            0.00000              ! Y_uSwitch		 - Wind speed to switch between Y_ErrThresh. If zero, only the first value of Y_ErrThresh is used [m/s]
133    Empty Line           N/A
134    PitchActSec          !------- Pitch Actuator Model -----------------------------------------------------
135    PA_CornerFreq        3.140000000000       ! PA_CornerFreq     - Pitch actuator bandwidth/cut-off frequency [rad/s]
136    PA_Damping           0.707000000000       ! PA_Damping        - Pitch actuator damping ratio [-, unused if PA_Mode = 1]
137    Empty Line          
138    ExtConSec            !------- External Controller Interface -----------------------------------------------------
139    DLL_FileName         "unused"             ! DLL_FileName        - Name/location of the dynamic library in the Bladed-DLL format
140    DLL_InFile           "unused"             ! DLL_InFile          - Name of input file sent to the DLL (-)
141    DLL_ProcName         "DISCON"             ! DLL_ProcName        - Name of procedure in DLL to be called (-) 
142    Empty Line          
143    ZeroMQSec            !------- ZeroMQ Interface ---------------------------------------------------------
144    ZMQ_CommAddress      "tcp://localhost:5555"   ! ZMQ_CommAddress     - Communication address for ZMQ server, (e.g. "tcp://localhost:5555")
145    ZMQ_UpdatePeriod     2                        ! ZMQ_UpdatePeriod    - Call ZeroMQ every [x] seconds, [s]
====== =================    ======================================================================================================================================================================================================

====== =================    ======================================================================================================================================================================================================
Modified in ROSCO 2.6.0
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
97     Y_ErrThresh          4.000000  8.000000  ! Y_ErrThresh    - Yaw error threshold/deadbands. Turbine begins to yaw when it passes this. If Y_uSwitch is zero, only the second value is used. [deg].
98     Y_Rate               0.00870              ! Y_Rate			- Yaw rate [rad/s]
99     Y_MErrSet            0.00000              ! Y_MErrSet		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]
====== =================    ======================================================================================================================================================================================================

====== =================    ======================================================================================================================================================================================================
Removed in ROSCO 2.6.0
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
96      Y_IPn               1                   ! Y_IPC_n			- Number of controller gains (yaw-by-IPC)
99      Y_IPC_omegaLP       0.20940             ! Y_IPC_omegaLP		- Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].
100     Y_IPC_zetaLP        1.00000             ! Y_IPC_zetaLP		- Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].
102     Y_omegaLPFast       0.20940             ! Y_omegaLPFast		- Corner frequency fast low pass filter, 1.0 [rad/s]
103     Y_omegaLPSlow       0.10470             ! Y_omegaLPSlow		- Corner frequency slow low pass filter, 1/60 [rad/s]
====== =================    ======================================================================================================================================================================================================

ROSCO v2.4.1 to ROSCO v2.5.0
-------------------------------
Two filter parameters were added to 
-  change the high pass filter in the floating feedback module
-  change the low pass filter of the wind speed estimator signal that is used in torque control

Open loop control inputs, users must specify:
-  The open loop input filename, an example can be found in Examples/Example_OL_Input.dat
-  Indices (columns) of values specified in OL_Filename

IPC
-  Proportional Control capabilities were added, 1P and 2P gains should be specified

====== =================    ======================================================================================================================================================================================================
Line    Input Name           Example Value
====== =================    ======================================================================================================================================================================================================
20     OL_Mode              0                   ! OL_Mode           - Open loop control mode {0: no open loop control, 1: open loop control vs. time, 2: open loop control vs. wind speed}
27     F_WECornerFreq       0.20944             ! F_WECornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].
29     F_FlHighPassFreq     0.01000             ! F_FlHighPassFreq  - Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s].
50     IPC_KP               0.000000  0.000000  ! IPC_KP			- Proportional gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]
125    OL_Filename          "14_OL_Input.dat"   ! OL_Filename       - Input file with open loop timeseries (absolute path or relative to this file)
126    Ind_Breakpoint       1                   ! Ind_Breakpoint    - The column in OL_Filename that contains the breakpoint (time if OL_Mode = 1)
127    Ind_BldPitch         2                   ! Ind_BldPitch      - The column in OL_Filename that contains the blade pitch input in rad
128    Ind_GenTq            3                   ! Ind_GenTq         - The column in OL_Filename that contains the generator torque in Nm
129    Ind_YawRate          4                   ! Ind_YawRate       - The column in OL_Filename that contains the generator torque in Nm
====== =================    ======================================================================================================================================================================================================
