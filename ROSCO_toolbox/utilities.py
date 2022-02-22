# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
"""
Class FileProcessing used to write out controller 
    parameter files need to run ROSCO

Methods:
-----------
run_openfast
write_DISCON
read_DISCON
write_rotor_performance
load_from_txt
DISCON_dict
list_check
"""
import datetime
import os
import numpy as np
import subprocess
import ROSCO_toolbox

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

def write_DISCON(turbine, controller, param_file='DISCON.IN', txt_filename='Cp_Ct_Cq.txt', rosco_vt = {}):
    """
    Print the controller parameters to the DISCON.IN input file for the generic controller

    Parameters:
    -----------
    turbine: class
                Turbine class containing turbine operation information (ref speeds, etc...)
    controller: class
                Controller class containing controller operation information (gains, etc...)
    param_file: str, optional
        filename for parameter input file, should be DISCON.IN
    txt_filename: str, optional
                    filename of rotor performance file
    """

    # Get ROSCO var tree if not provided
    if not rosco_vt:
        rosco_vt = DISCON_dict(turbine, controller, txt_filename)

    print('Writing new controller parameter file parameter file: %s.' % param_file)
    # Should be obvious what's going on here...
    file = open(param_file,'w')
    file.write('! Controller parameter input file for the %s wind turbine\n' % turbine.TurbineName)
    file.write('!    - File written using ROSCO version {} controller tuning logic on {}\n'.format(ROSCO_toolbox.__version__, now.strftime('%m/%d/%y')))
    file.write('\n')
    file.write('!------- DEBUG ------------------------------------------------------------\n')
    file.write('{0:<12d}        ! LoggingLevel		- {{0: write no debug files, 1: write standard output .dbg-file, 2: write standard output .dbg-file and complete avrSWAP-array .dbg2-file}}\n'.format(int(rosco_vt['LoggingLevel'])))
    file.write('\n')
    file.write('!------- CONTROLLER FLAGS -------------------------------------------------\n')
    file.write('{0:<12d}        ! F_LPFType			- {{1: first-order low-pass filter, 2: second-order low-pass filter}}, [rad/s] (currently filters generator speed and pitch control signals\n'.format(int(rosco_vt['F_LPFType'])))
    file.write('{0:<12d}        ! F_NotchType		- Notch on the measured generator speed and/or tower fore-aft motion (for floating) {{0: disable, 1: generator speed, 2: tower-top fore-aft motion, 3: generator speed and tower-top fore-aft motion}}\n'.format(int(rosco_vt['F_NotchType'])))
    file.write('{0:<12d}        ! IPC_ControlMode	- Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {{0: off, 1: 1P reductions, 2: 1P+2P reductions}}\n'.format(int(rosco_vt['IPC_ControlMode'])))
    file.write('{0:<12d}        ! VS_ControlMode	- Generator torque control mode in above rated conditions {{0: constant torque, 1: constant power, 2: TSR tracking PI control with constant torque, 3: TSR tracking PI control with constant power}}\n'.format(int(rosco_vt['VS_ControlMode'])))
    file.write('{0:<12d}        ! PC_ControlMode    - Blade pitch control mode {{0: No pitch, fix to fine pitch, 1: active PI blade pitch control}}\n'.format(int(rosco_vt['PC_ControlMode'])))
    file.write('{0:<12d}        ! Y_ControlMode		- Yaw control mode {{0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC}}\n'.format(int(rosco_vt['Y_ControlMode'])))
    file.write('{0:<12d}        ! SS_Mode           - Setpoint Smoother mode {{0: no setpoint smoothing, 1: introduce setpoint smoothing}}\n'.format(int(rosco_vt['SS_Mode'])))
    file.write('{0:<12d}        ! WE_Mode           - Wind speed estimator mode {{0: One-second low pass filtered hub height wind speed, 1: Immersion and Invariance Estimator, 2: Extended Kalman Filter}}\n'.format(int(rosco_vt['WE_Mode'])))
    file.write('{0:<12d}        ! PS_Mode           - Pitch saturation mode {{0: no pitch saturation, 1: implement pitch saturation}}\n'.format(int(rosco_vt['PS_Mode'])))
    file.write('{0:<12d}        ! SD_Mode           - Shutdown mode {{0: no shutdown procedure, 1: pitch to max pitch at shutdown}}\n'.format(int(rosco_vt['SD_Mode'])))
    file.write('{0:<12d}        ! Fl_Mode           - Floating specific feedback mode {{0: no nacelle velocity feedback, 1: feed back translational velocity, 2: feed back rotational veloicty}}\n'.format(int(rosco_vt['Fl_Mode'])))
    file.write('{0:<12d}        ! Flp_Mode          - Flap control mode {{0: no flap control, 1: steady state flap angle, 2: Proportional flap control, 2: Cyclic (1P) flap control}}\n'.format(int(rosco_vt['Flp_Mode'])))
    file.write('{0:<12d}        ! OL_Mode           - Open loop control mode {{0: no open loop control, 1: open loop control vs. time, 2: open loop control vs. wind speed}}\n'.format(int(rosco_vt['OL_Mode'])))
    file.write('\n')
    file.write('!------- FILTERS ----------------------------------------------------------\n') 
    file.write('{:<13.5f}       ! F_LPFCornerFreq	- Corner frequency (-3dB point) in the low-pass filters, [rad/s]\n'.format(rosco_vt['F_LPFCornerFreq'])) 
    file.write('{:<13.5f}       ! F_LPFDamping		- Damping coefficient {{used only when F_FilterType = 2}} [-]\n'.format(rosco_vt['F_LPFDamping']))
    file.write('{:<13.5f}       ! F_NotchCornerFreq	- Natural frequency of the notch filter, [rad/s]\n'.format(rosco_vt['F_NotchCornerFreq']))
    file.write('{} ! F_NotchBetaNumDen	- Two notch damping values (numerator and denominator, resp) - determines the width and depth of the notch, [-]\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['F_NotchBetaNumDen'][i]) for i in range(len(rosco_vt['F_NotchBetaNumDen'])))))
    file.write('{:<13.5f}       ! F_SSCornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother, [rad/s].\n'.format(rosco_vt['F_SSCornerFreq']))
    file.write('{:<13.5f}       ! F_WECornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].\n'.format(rosco_vt['F_WECornerFreq']))
    file.write('{} ! F_FlCornerFreq    - Natural frequency and damping in the second order low pass filter of the tower-top fore-aft motion for floating feedback control [rad/s, -].\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['F_FlCornerFreq'][i]) for i in range(len(rosco_vt['F_FlCornerFreq'])))))
    file.write('{:<13.5f}       ! F_FlHighPassFreq    - Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s].\n'.format(rosco_vt['F_FlHighPassFreq']))
    file.write('{} ! F_FlpCornerFreq   - Corner frequency and damping in the second order low pass filter of the blade root bending moment for flap control [rad/s, -].\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['F_FlpCornerFreq'][i]) for i in range(len(rosco_vt['F_FlpCornerFreq'])))))
    
    file.write('\n')
    file.write('!------- BLADE PITCH CONTROL ----------------------------------------------\n')
    file.write('{:<11d}         ! PC_GS_n			- Amount of gain-scheduling table entries\n'.format(int(rosco_vt['PC_GS_n'])))
    file.write('{}              ! PC_GS_angles	    - Gain-schedule table: pitch angles [rad].\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['PC_GS_angles'][i]) for i in range(len(rosco_vt['PC_GS_angles'])))))            
    file.write('{}              ! PC_GS_KP		- Gain-schedule table: pitch controller kp gains [s].\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['PC_GS_KP'][i]) for i in range(len(rosco_vt['PC_GS_KP'])))))
    file.write('{}              ! PC_GS_KI		- Gain-schedule table: pitch controller ki gains [-].\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['PC_GS_KI'][i]) for i in range(len(rosco_vt['PC_GS_KI'])))))
    file.write('{}              ! PC_GS_KD			- Gain-schedule table: pitch controller kd gains\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['PC_GS_KD'][i]) for i in range(len(rosco_vt['PC_GS_KD'])))))
    file.write('{}              ! PC_GS_TF			- Gain-schedule table: pitch controller tf gains (derivative filter)\n'.format(''.join('{:<4.6f}  '.format(rosco_vt['PC_GS_TF'][i]) for i in range(len(rosco_vt['PC_GS_TF'])))))
    file.write('{:<014.5f}      ! PC_MaxPit			- Maximum physical pitch limit, [rad].\n'.format(rosco_vt['PC_MaxPit']))
    file.write('{:<014.5f}      ! PC_MinPit			- Minimum physical pitch limit, [rad].\n'.format(rosco_vt['PC_MinPit']))
    file.write('{:<014.5f}      ! PC_MaxRat			- Maximum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(rosco_vt['PC_MaxRat']))
    file.write('{:<014.5f}      ! PC_MinRat			- Minimum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(rosco_vt['PC_MinRat']))
    file.write('{:<014.5f}      ! PC_RefSpd			- Desired (reference) HSS speed for pitch controller, [rad/s].\n'.format(rosco_vt['PC_RefSpd']))
    file.write('{:<014.5f}      ! PC_FinePit		- Record 5: Below-rated pitch angle set-point, [rad]\n'.format(rosco_vt['PC_FinePit']))
    file.write('{:<014.5f}      ! PC_Switch			- Angle above lowest minimum pitch angle for switch, [rad]\n'.format(rosco_vt['PC_Switch']))
    file.write('\n')
    file.write('!------- INDIVIDUAL PITCH CONTROL -----------------------------------------\n')
    file.write('{:<13.1f}       ! IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from IPC), [rad]\n'.format(rosco_vt['IPC_IntSat'])) # Hardcode to 5 degrees
    file.write('{} ! IPC_KP			- Proportional gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n'.format(''.join('{:<4.3e} '.format(rosco_vt['IPC_KP'][i]) for i in range(len(rosco_vt['IPC_KP'])))))
    file.write('{} ! IPC_KI			- Integral gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n'.format(''.join('{:<4.3e} '.format(rosco_vt['IPC_KI'][i]) for i in range(len(rosco_vt['IPC_KI'])))))
    file.write('{} ! IPC_aziOffset		- Phase offset added to the azimuth angle for the individual pitch controller, [rad]. \n'.format(''.join('{:<4.6f}  '.format(rosco_vt['IPC_aziOffset'][i]) for i in range(len(rosco_vt['IPC_aziOffset'])))))
    file.write('{:<13.1f}       ! IPC_CornerFreqAct - Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {{0: Disable}}, [rad/s]\n'.format(rosco_vt['IPC_CornerFreqAct']))
    file.write('\n')
    file.write('!------- VS TORQUE CONTROL ------------------------------------------------\n')
    file.write('{:<014.5f}      ! VS_GenEff			- Generator efficiency mechanical power -> electrical power, [should match the efficiency defined in the generator properties!], [%]\n'.format(rosco_vt['VS_GenEff']))
    file.write('{:<014.5f}      ! VS_ArSatTq		- Above rated generator torque PI control saturation, [Nm]\n'.format(rosco_vt['VS_ArSatTq']))
    file.write('{:<014.5f}      ! VS_MaxRat			- Maximum torque rate (in absolute value) in torque controller, [Nm/s].\n'.format(rosco_vt['VS_MaxRat']))
    file.write('{:<014.5f}      ! VS_MaxTq			- Maximum generator torque in Region 3 (HSS side), [Nm].\n'.format(rosco_vt['VS_MaxTq']))
    file.write('{:<014.5f}      ! VS_MinTq			- Minimum generator torque (HSS side), [Nm].\n'.format(rosco_vt['VS_MinTq']))
    file.write('{:<014.5f}      ! VS_MinOMSpd		- Minimum generator speed [rad/s]\n'.format(rosco_vt['VS_MinOMSpd']))
    file.write('{:<014.5f}      ! VS_Rgn2K			- Generator torque constant in Region 2 (HSS side), [Nm/(rad/s)^2]\n'.format(rosco_vt['VS_Rgn2K']))
    file.write('{:<014.5f}      ! VS_RtPwr			- Wind turbine rated power [W]\n'.format(rosco_vt['VS_RtPwr']))
    file.write('{:<014.5f}      ! VS_RtTq			- Rated torque, [Nm].\n'.format(rosco_vt['VS_RtTq']))
    file.write('{:<014.5f}      ! VS_RefSpd			- Rated generator speed [rad/s]\n'.format(rosco_vt['VS_RefSpd']))
    file.write('{:<11d}         ! VS_n				- Number of generator PI torque controller gains\n'.format(int(rosco_vt['VS_n'])))
    file.write('{:<014.5f}      ! VS_KP				- Proportional gain for generator PI torque controller [-]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(rosco_vt['VS_KP']))
    file.write('{:<014.5f}      ! VS_KI				- Integral gain for generator PI torque controller [s]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(rosco_vt['VS_KI']))
    file.write('{:<13.2f}       ! VS_TSRopt			- Power-maximizing region 2 tip-speed-ratio [rad].\n'.format(rosco_vt['VS_TSRopt']))
    file.write('\n')
    file.write('!------- SETPOINT SMOOTHER ---------------------------------------------\n')
    file.write('{:<13.5f}       ! SS_VSGain         - Variable speed torque controller setpoint smoother gain, [-].\n'.format(rosco_vt['SS_VSGain']))
    file.write('{:<13.5f}       ! SS_PCGain         - Collective pitch controller setpoint smoother gain, [-].\n'.format(rosco_vt['SS_PCGain']))
    file.write('\n')
    file.write('!------- WIND SPEED ESTIMATOR ---------------------------------------------\n')
    file.write('{:<13.3f}       ! WE_BladeRadius	- Blade length (distance from hub center to blade tip), [m]\n'.format(rosco_vt['WE_BladeRadius']))
    file.write('{:<11d}         ! WE_CP_n			- Amount of parameters in the Cp array\n'.format(int(rosco_vt['WE_CP_n'])))
    file.write('{}              ! WE_CP - Parameters that define the parameterized CP(lambda) function\n'.format(rosco_vt['WE_CP']))
    file.write('{:<13.1f}		! WE_Gamma			- Adaption gain of the wind speed estimator algorithm [m/rad]\n'.format(rosco_vt['WE_Gamma']))
    file.write('{:<13.1f}       ! WE_GearboxRatio	- Gearbox ratio [>=1],  [-]\n'.format(rosco_vt['WE_GearboxRatio']))
    file.write('{:<14.5f}       ! WE_Jtot			- Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, [kg m^2]\n'.format(rosco_vt['WE_Jtot']))
    file.write('{:<13.3f}       ! WE_RhoAir			- Air density, [kg m^-3]\n'.format(rosco_vt['WE_RhoAir']))
    file.write(      '"{}"      ! PerfFileName      - File containing rotor performance tables (Cp,Ct,Cq) (absolute path or relative to this file)\n'.format(rosco_vt['PerfFileName']))
    file.write('{:<7d} {:<10d}  ! PerfTableSize     - Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios\n'.format(int(rosco_vt['PerfTableSize'][0]),int(rosco_vt['PerfTableSize'][1])))
    file.write('{:<11d}         ! WE_FOPoles_N      - Number of first-order system poles used in EKF\n'.format(int(rosco_vt['WE_FOPoles_N'])))
    file.write('{}              ! WE_FOPoles_v      - Wind speeds corresponding to first-order system poles [m/s]\n'.format(''.join('{:<4.4f} '.format(rosco_vt['WE_FOPoles_v'][i]) for i in range(len(rosco_vt['WE_FOPoles_v'])))))
    file.write('{}              ! WE_FOPoles        - First order system poles [1/s]\n'.format(''.join('{:<10.8f} '.format(rosco_vt['WE_FOPoles'][i]) for i in range(len(rosco_vt['WE_FOPoles'])))))
    file.write('\n')
    file.write('!------- YAW CONTROL ------------------------------------------------------\n')
    file.write('{:<13.5f}       ! Y_ErrThresh		- Yaw error threshold. Turbine begins to yaw when it passes this. [rad^2 s]\n'.format(rosco_vt['Y_ErrThresh']))
    file.write('{:<13.5f}       ! Y_IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]\n'.format(rosco_vt['Y_IPC_IntSat']))
    file.write('{:<11d}         ! Y_IPC_n			- Number of controller gains (yaw-by-IPC)\n'.format(int(rosco_vt['Y_IPC_n'])))
    file.write('{:<13.5f}       ! Y_IPC_KP			- Yaw-by-IPC proportional controller gain Kp\n'.format(rosco_vt['Y_IPC_KP']))
    file.write('{:<13.5f}       ! Y_IPC_KI			- Yaw-by-IPC integral controller gain Ki\n'.format(rosco_vt['Y_IPC_KI']))
    file.write('{:<13.5f}       ! Y_IPC_omegaLP		- Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].\n'.format(rosco_vt['Y_IPC_omegaLP']))
    file.write('{:<13.5f}       ! Y_IPC_zetaLP		- Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].\n'.format(rosco_vt['Y_IPC_zetaLP']))
    file.write('{:<13.5f}       ! Y_MErrSet			- Yaw alignment error, set point [rad]\n'.format(rosco_vt['Y_MErrSet']))
    file.write('{:<13.5f}       ! Y_omegaLPFast		- Corner frequency fast low pass filter, 1.0 [rad/s]\n'.format(rosco_vt['Y_omegaLPFast']))
    file.write('{:<13.5f}       ! Y_omegaLPSlow		- Corner frequency slow low pass filter, 1/60 [rad/s]\n'.format(rosco_vt['Y_omegaLPSlow']))
    file.write('{:<13.5f}       ! Y_Rate			- Yaw rate [rad/s]\n'.format(rosco_vt['Y_Rate']))
    file.write('\n')
    file.write('!------- TOWER FORE-AFT DAMPING -------------------------------------------\n')
    file.write('{:<11d}         ! FA_KI				- Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m] - !NJA - Make this a flag\n'.format(int(rosco_vt['FA_KI'] )))
    file.write('{:<13.1f}       ! FA_HPFCornerFreq	- Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]\n'.format(rosco_vt['FA_HPFCornerFreq'] ))
    file.write('{:<13.1f}       ! FA_IntSat			- Integrator saturation (maximum signal amplitude contribution to pitch from FA damper), [rad]\n'.format(rosco_vt['FA_IntSat'] ))
    file.write('\n')
    file.write('!------- MINIMUM PITCH SATURATION -------------------------------------------\n')
    file.write('{:<11d}         ! PS_BldPitchMin_N  - Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)\n'.format(int(rosco_vt['PS_BldPitchMin_N'])))
    file.write('{}              ! PS_WindSpeeds     - Wind speeds corresponding to minimum blade pitch angles [m/s]\n'.format(''.join('{:<4.4f} '.format(rosco_vt['PS_WindSpeeds'][i]) for i in range(len(rosco_vt['PS_WindSpeeds'])))))
    file.write('{}              ! PS_BldPitchMin    - Minimum blade pitch angles [rad]\n'.format(''.join('{:<10.8f} '.format(rosco_vt['PS_BldPitchMin'][i]) for i in range(len(rosco_vt['PS_BldPitchMin'])))))
    file.write('\n')
    file.write('!------- SHUTDOWN -----------------------------------------------------------\n')
    file.write('{:<014.5f}      ! SD_MaxPit         - Maximum blade pitch angle to initiate shutdown, [rad]\n'.format(rosco_vt['SD_MaxPit']))
    file.write('{:<014.5f}      ! SD_CornerFreq     - Cutoff Frequency for first order low-pass filter for blade pitch angle, [rad/s]\n'.format(rosco_vt['SD_CornerFreq']))
    file.write('\n')
    file.write('!------- Floating -----------------------------------------------------------\n')
    if rosco_vt['Fl_Mode'] == 2:
        floatstr = 'pitching'
    else:
        floatstr = 'velocity'
    file.write('{:<014.5f}      ! Fl_Kp             - Nacelle {} proportional feedback gain [s]\n'.format(rosco_vt['Fl_Kp'], floatstr))
    file.write('\n')
    file.write('!------- FLAP ACTUATION -----------------------------------------------------\n')
    file.write('{:<014.5f}      ! Flp_Angle         - Initial or steady state flap angle [rad]\n'.format(rosco_vt['Flp_Angle']))
    file.write('{:<014.8e}      ! Flp_Kp            - Blade root bending moment proportional gain for flap control [s]\n'.format(rosco_vt['Flp_Kp']))
    file.write('{:<014.8e}      ! Flp_Ki            - Flap displacement integral gain for flap control [-]\n'.format(rosco_vt['Flp_Ki']))
    file.write('{:<014.5f}      ! Flp_MaxPit        - Maximum (and minimum) flap pitch angle [rad]\n'.format(rosco_vt['Flp_MaxPit']))
    file.write('\n')
    file.write('!------- Open Loop Control -----------------------------------------------------\n')
    file.write('"{}"            ! OL_Filename       - Input file with open loop timeseries (absolute path or relative to this file)\n'.format(rosco_vt['OL_Filename']))
    file.write('{0:<12d}        ! Ind_Breakpoint    - The column in OL_Filename that contains the breakpoint (time if OL_Mode = 1)\n'.format(int(rosco_vt['Ind_Breakpoint'])))
    file.write('{0:<12d}        ! Ind_BldPitch      - The column in OL_Filename that contains the blade pitch input in rad\n'.format(int(rosco_vt['Ind_BldPitch'])))
    file.write('{0:<12d}        ! Ind_GenTq         - The column in OL_Filename that contains the generator torque in Nm\n'.format(int(rosco_vt['Ind_GenTq'])))
    file.write('{0:<12d}        ! Ind_YawRate       - The column in OL_Filename that contains the generator torque in Nm\n'.format(int(rosco_vt['Ind_YawRate'])))
    file.close()
    
    # Write Open loop input
    if rosco_vt['OL_Mode'] and hasattr(controller, 'OpenLoop'):
        write_ol_control(controller)

def read_DISCON(DISCON_filename):
    '''
    Read the DISCON input file.

    Parameters:
    ----------
    DISCON_filename: string
        Name of DISCON input file to read
    
    Returns:
    --------
    DISCON_in: Dict
        Dictionary containing input parameters from DISCON_in, organized by parameter name
    '''
    
    DISCON_in = {}
    with open(DISCON_filename) as discon:
        for line in discon:

            # Skip whitespace and comment lines
            if (line[0] != '!') == (len(line.strip()) != 0):
                
                if (line.split()[1] != '!'):    # Array valued entries
                    array_length = line.split().index('!')
                    param = line.split()[array_length+1]
                    values = np.array( [float(x) for x in line.split()[:array_length]] )
                    DISCON_in[param] = values
                else:                           # All other entries
                    param = line.split()[2]
                    value = line.split()[0]
                    # Remove printed quotations if string is in quotes
                    if (value[0] == '"') or (value[0] == "'"):
                        value = value[1:-1]
                    else:
                        value = float(value)
                    DISCON_in[param] = value

    return DISCON_in
    

def write_rotor_performance(turbine,txt_filename='Cp_Ct_Cq.txt'):
    '''
    Write text file containing rotor performance data

    Parameters:
    ------------
        txt_filename: str, optional
                        Desired output filename to print rotor performance data. Default is Cp_Ct_Cq.txt
    '''
    print('Writing rotor performance text file: {}'.format(txt_filename))
    file = open(txt_filename,'w')
    # Headerlines
    file.write('# ----- Rotor performance tables for the {} wind turbine ----- \n'.format(turbine.TurbineName))
    file.write('# ------------ Written on {} using the ROSCO toolbox ------------ \n\n'.format(now.strftime('%b-%d-%y')))

    # Pitch angles, TSR, and wind speed
    file.write('# Pitch angle vector, {} entries - x axis (matrix columns) (deg)\n'.format(len(turbine.Cp.pitch_initial_rad)))
    for i in range(len(turbine.Cp.pitch_initial_rad)):
        file.write('{:0.4}   '.format(turbine.Cp.pitch_initial_rad[i] * rad2deg))
    file.write('\n# TSR vector, {} entries - y axis (matrix rows) (-)\n'.format(len(turbine.TSR_initial)))
    for i in range(len(turbine.TSR_initial)):
        file.write('{:0.4}    '.format(turbine.Cp.TSR_initial[i]))
    file.write('\n# Wind speed vector - z axis (m/s)\n')
    file.write('{:0.4}    '.format(turbine.v_rated))
    file.write('\n')
    
    # Cp
    file.write('\n# Power coefficient\n\n')
    for i in range(len(turbine.Cp.TSR_initial)):
        for j in range(len(turbine.Cp.pitch_initial_rad)):
            file.write('{0:.6f}   '.format(turbine.Cp_table[i,j]))
        file.write('\n')
    file.write('\n')
    
    # Ct
    file.write('\n#  Thrust coefficient\n\n')
    for i in range(len(turbine.Ct.TSR_initial)):
        for j in range(len(turbine.Ct.pitch_initial_rad)):
            file.write('{0:.6f}   '.format(turbine.Ct_table[i,j]))
        file.write('\n')
    file.write('\n')
    
    # Cq
    file.write('\n# Torque coefficient\n\n')
    for i in range(len(turbine.Cq.TSR_initial)):
        for j in range(len(turbine.Cq.pitch_initial_rad)):
            file.write('{0:.6f}   '.format(turbine.Cq_table[i,j]))
        file.write('\n')
    file.write('\n')
    file.close()


def load_from_txt(txt_filename):
    '''
    Load rotor performance data from a *.txt file. 

    Parameters:
    -----------
        txt_filename: str
                        Filename of the text containing the Cp, Ct, and Cq data. This should be in the format printed by the write_rotorperformance function
    '''
    print('Loading rotor performace data from text file:', txt_filename)

    with open(txt_filename) as pfile:
        for line in pfile:
            # Read Blade Pitch Angles (degrees)
            if 'Pitch angle' in line:
                pitch_initial = np.array([float(x) for x in pfile.readline().strip().split()])
                pitch_initial_rad = pitch_initial * deg2rad             # degrees to rad            -- should this be conditional?

            # Read Tip Speed Ratios (rad)
            if 'TSR' in line:
                TSR_initial = np.array([float(x) for x in pfile.readline().strip().split()])
            
            # Read Power Coefficients
            if 'Power' in line:
                pfile.readline()
                Cp = np.empty((len(TSR_initial),len(pitch_initial)))
                for tsr_i in range(len(TSR_initial)):
                    Cp[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])
            
            # Read Thrust Coefficients
            if 'Thrust' in line:
                pfile.readline()
                Ct = np.empty((len(TSR_initial),len(pitch_initial)))
                for tsr_i in range(len(TSR_initial)):
                    Ct[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])

            # Read Torque Coefficients
            if 'Torque' in line:
                pfile.readline()
                Cq = np.empty((len(TSR_initial),len(pitch_initial)))
                for tsr_i in range(len(TSR_initial)):
                    Cq[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])

        # return pitch_initial_rad TSR_initial Cp Ct Cq
        # Store necessary metrics for analysis and tuning
        # self.pitch_initial_rad = pitch_initial_rad
        # self.TSR_initial = TSR_initial
        # self.Cp_table = Cp
        # self.Ct_table = Ct 
        # self.Cq_table = Cq
        return pitch_initial_rad, TSR_initial, Cp, Ct, Cq


def DISCON_dict(turbine, controller, txt_filename=None):
    '''
    Convert the turbine and controller objects to a dictionary organized by the parameter names 
    that are defined in the DISCON.IN file.

    Parameters
    ----------
    turbine: obj
        Turbine object output from the turbine class
    controller: obj
        Controller object output from the controller class
    txt_filename: string, optional
        Name of rotor performance filename
    '''
    DISCON_dict = {}
    # ------- DEBUG -------
    DISCON_dict['LoggingLevel']	    = int(controller.LoggingLevel)
    # ------- CONTROLLER FLAGS -------
    DISCON_dict['F_LPFType']	    = int(controller.F_LPFType)
    DISCON_dict['F_NotchType']		= int(controller.F_NotchType)
    DISCON_dict['IPC_ControlMode']	= int(controller.IPC_ControlMode)
    DISCON_dict['VS_ControlMode']	= int(controller.VS_ControlMode)
    DISCON_dict['PC_ControlMode']   = int(controller.PC_ControlMode)
    DISCON_dict['Y_ControlMode']	= int(controller.Y_ControlMode)
    DISCON_dict['SS_Mode']          = int(controller.SS_Mode)
    DISCON_dict['WE_Mode']          = int(controller.WE_Mode)
    DISCON_dict['PS_Mode']          = int(controller.PS_Mode > 0)
    DISCON_dict['SD_Mode']          = int(controller.SD_Mode)
    DISCON_dict['Fl_Mode']          = int(controller.Fl_Mode)
    DISCON_dict['Flp_Mode']         = int(controller.Flp_Mode)
    DISCON_dict['OL_Mode']          = int(controller.OL_Mode)
    # ------- FILTERS -------
    DISCON_dict['F_LPFCornerFreq']	    = turbine.bld_edgewise_freq * 1/4
    DISCON_dict['F_LPFDamping']		    = controller.F_LPFDamping
    if controller.Flp_Mode > 0:
        DISCON_dict['F_NotchCornerFreq'] = turbine.bld_flapwise_freq
        DISCON_dict['F_NotchBetaNumDen'] = [0.0, 0.5]
    else:
        DISCON_dict['F_NotchCornerFreq'] = controller.twr_freq
        DISCON_dict['F_NotchBetaNumDen'] = [0.0, 0.25]
    DISCON_dict['F_WECornerFreq'] = controller.f_we_cornerfreq
    DISCON_dict['F_SSCornerFreq'] = controller.f_ss_cornerfreq
    DISCON_dict['F_FlHighPassFreq'] = controller.f_fl_highpassfreq
    DISCON_dict['F_FlCornerFreq'] = [controller.ptfm_freq, 1.0]
    DISCON_dict['F_FlpCornerFreq'] = [turbine.bld_flapwise_freq*3, 1.0]
    DISCON_dict['F_WECornerFreq']       = controller.f_we_cornerfreq
    DISCON_dict['F_SSCornerFreq']       = controller.f_ss_cornerfreq
    DISCON_dict['F_FlHighPassFreq']     = controller.f_fl_highpassfreq
    DISCON_dict['F_FlCornerFreq']       = [controller.ptfm_freq, 1.0]
    # ------- BLADE PITCH CONTROL -------
    DISCON_dict['PC_GS_n']			= len(controller.pitch_op_pc)
    DISCON_dict['PC_GS_angles']	    = controller.pitch_op_pc
    DISCON_dict['PC_GS_KP']		    = controller.pc_gain_schedule.Kp
    DISCON_dict['PC_GS_KI']		    = controller.pc_gain_schedule.Ki
    DISCON_dict['PC_GS_KD']			= [0.0 for i in range(len(controller.pc_gain_schedule.Ki))]
    DISCON_dict['PC_GS_TF']			= [0.0 for i in range(len(controller.pc_gain_schedule.Ki))]
    DISCON_dict['PC_MaxPit']		= controller.max_pitch
    DISCON_dict['PC_MinPit']		= controller.min_pitch
    DISCON_dict['PC_MaxRat']		= turbine.max_pitch_rate
    DISCON_dict['PC_MinRat']		= turbine.min_pitch_rate
    DISCON_dict['PC_RefSpd']		= turbine.rated_rotor_speed*turbine.Ng
    DISCON_dict['PC_FinePit']		= controller.min_pitch
    DISCON_dict['PC_Switch']		= 1 * deg2rad
    # ------- INDIVIDUAL PITCH CONTROL -------
    DISCON_dict['IPC_IntSat']		= 0.087266
    DISCON_dict['IPC_KP']           = [controller.Kp_ipc1p, controller.Kp_ipc2p]
    DISCON_dict['IPC_KI']           = [controller.Ki_ipc1p, controller.Ki_ipc2p]
    DISCON_dict['IPC_aziOffset']	= [0.0, 0.0]
    DISCON_dict['IPC_CornerFreqAct'] = 0.0
    # ------- VS TORQUE CONTROL -------
    DISCON_dict['VS_GenEff']		= turbine.GenEff
    DISCON_dict['VS_ArSatTq']		= turbine.rated_torque
    DISCON_dict['VS_MaxRat']		= turbine.max_torque_rate
    DISCON_dict['VS_MaxTq']			= turbine.max_torque
    DISCON_dict['VS_MinTq']			= 0.0
    DISCON_dict['VS_MinOMSpd']		= controller.vs_minspd  * turbine.Ng
    DISCON_dict['VS_Rgn2K']			= controller.vs_rgn2K
    DISCON_dict['VS_RtPwr']			= turbine.rated_power
    DISCON_dict['VS_RtTq']			= turbine.rated_torque
    DISCON_dict['VS_RefSpd']		= controller.vs_refspd
    DISCON_dict['VS_n']				= 1
    DISCON_dict['VS_KP']			= controller.vs_gain_schedule.Kp[-1]
    DISCON_dict['VS_KI']			= controller.vs_gain_schedule.Ki[-1]
    DISCON_dict['VS_TSRopt']		= turbine.TSR_operational
    # ------- SETPOINT SMOOTHER -------
    DISCON_dict['SS_VSGain']         = controller.ss_vsgain
    DISCON_dict['SS_PCGain']         = controller.ss_pcgain
    # ------- WIND SPEED ESTIMATOR -------
    DISCON_dict['WE_BladeRadius']	= turbine.rotor_radius
    DISCON_dict['WE_CP_n']			= 1
    DISCON_dict['WE_CP']            = 0
    DISCON_dict['WE_Gamma']			= 0.0
    DISCON_dict['WE_GearboxRatio']	= turbine.Ng
    DISCON_dict['WE_Jtot']			= turbine.J
    DISCON_dict['WE_RhoAir']		= turbine.rho
    DISCON_dict['PerfFileName']     = txt_filename
    DISCON_dict['PerfTableSize']    = [len(turbine.Cp.pitch_initial_rad),len(turbine.Cp.TSR_initial)]
    DISCON_dict['WE_FOPoles_N']     = len(controller.A)
    DISCON_dict['WE_FOPoles_v']     = controller.v
    DISCON_dict['WE_FOPoles']       = controller.A
    # ------- YAW CONTROL -------
    DISCON_dict['Y_ErrThresh']		= 0.1396
    DISCON_dict['Y_IPC_IntSat']		= 0.0
    DISCON_dict['Y_IPC_n']			= 1
    DISCON_dict['Y_IPC_KP']			= 0.0
    DISCON_dict['Y_IPC_KI']			= 0.0
    DISCON_dict['Y_IPC_omegaLP']    = 0.2094
    DISCON_dict['Y_IPC_zetaLP']		= 1
    DISCON_dict['Y_MErrSet']		= 0.0
    DISCON_dict['Y_omegaLPFast']	= 0.2094
    DISCON_dict['Y_omegaLPSlow']	= 0.1047
    DISCON_dict['Y_Rate']			= 0.0052
    # ------- TOWER FORE-AFT DAMPING -------
    DISCON_dict['FA_KI']                = -1
    DISCON_dict['FA_HPFCornerFreq'] = 0.0
    DISCON_dict['FA_IntSat']		 = 0.0
    # ------- MINIMUM PITCH SATURATION -------
    DISCON_dict['PS_BldPitchMin_N'] = len(controller.ps_min_bld_pitch)
    DISCON_dict['PS_WindSpeeds']    = controller.v
    DISCON_dict['PS_BldPitchMin']   = controller.ps_min_bld_pitch
    # ------- SHUTDOWN -------
    DISCON_dict['SD_MaxPit']        = controller.sd_maxpit
    DISCON_dict['SD_CornerFreq']    = controller.f_sd_cornerfreq
    # ------- Floating -------
    DISCON_dict['Fl_Kp']            = controller.Kp_float
    # ------- FLAP ACTUATION -------
    DISCON_dict['Flp_Angle']        = controller.flp_angle
    DISCON_dict['Flp_Kp']           = controller.Kp_flap[-1]
    DISCON_dict['Flp_Ki']           = controller.Ki_flap[-1]
    DISCON_dict['Flp_MaxPit']       = controller.flp_maxpit
    # ------- Open Loop Control -------
    DISCON_dict['OL_Filename']     = controller.OL_Filename
    DISCON_dict['Ind_Breakpoint']  = controller.OL_Ind_Breakpoint
    DISCON_dict['Ind_BldPitch']    = controller.OL_Ind_BldPitch
    DISCON_dict['Ind_GenTq']       = controller.OL_Ind_GenTq
    DISCON_dict['Ind_YawRate']     = controller.OL_Ind_YawRate

    return DISCON_dict


def run_openfast(fast_dir, fastcall='openfast', fastfile=None, chdir=True, restart=False):
    '''
    Runs a openfast openfast simulation.
    
    NOTE: Enabling chdir can help reduce file path errors in the DISCON.IN file.

    Parameters:
    ------------
        fast_dir: string
                Name of OpenFAST directory containing input files.
        fast_file: string
                Name of OpenFAST directory containing input files.
        fastcall: string, optional
                Line used to call openfast when executing from the terminal.
        fastfile: string, optional
                Filename for *.fst input file. Function will find *.fst if not provided.
        chdir: bool, optional
                Change directory to openfast model directory before running.
    '''

    # Define OpenFAST input filename
    if not fastfile:
        for file in os.listdir(fast_dir):
            if file.endswith('.fst'):
                fastfile = file
    print('Using {} to run OpenFAST simulation'.format(fastfile))

    if chdir: # Change cwd before calling OpenFAST -- note: This is an artifact of needing to call OpenFAST from the same directory as DISCON.IN
        cwd = fast_dir
    else:
        cwd = None

    print('Running OpenFAST simulation for {} through the ROSCO toolbox...'.format(fastfile))
        # os.system('{} {}'.format(fastcall, os.path.join(fastfile)))
    if restart:
        subprocess.run([fastcall,'-restart', os.path.join(fastfile)], check=True, cwd=cwd)
    else:
        subprocess.run([fastcall, os.path.join(fastfile)], check=True, cwd=cwd)
    print('OpenFAST simulation complete.')


def list_check(x, return_bool=True):
    '''
    Check if the input is list-like or not

    Parameters:
    -----------
        x: int, float, list, or np.ndarray
            input to check
        return_bool: bool
            if true, returns True or False 

    '''
    if isinstance(x, (int, float)):
        y = x
        is_list = False
    elif isinstance(x, list):
        if len(x) == 1:
            y = x[0]
            is_list = False
        else:
            y = x
            is_list = True
    elif isinstance(x, np.ndarray):
        if x.size == 1:
            is_list = False
            y = float(x)
        else:
            is_list = True
            y = x
    else:
        raise AttributeError('Cannot run list_check for variable of type: {}'.format(type(x)))

    if return_bool:
        return is_list
    else:
        return y
