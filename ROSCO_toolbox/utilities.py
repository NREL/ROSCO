# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import datetime
import os
import numpy as np

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class UseOpenFAST():
    ''' 
    A collection of utilities that may be useful for using the tools made accessbile in this toolbox with OpenFAST

    Methods:
    --------
    run_openfast
    plot_fast_out
    read_fast_out
    '''
    def __init__(self):
        pass

    def run_openfast(self,fast_dir,fastcall='OpenFAST',fastfile=None,):
        '''
        Runs a openfast openfast simulation 
        Parameters:
        ------------
            fast_dir: string
                    Name of OpenFAST directory containing input files.
            fastcall: string, optional
                    Line used to call openfast when executing from the terminal.
            fastfile: string, optional
                    Filename for *.fst input file. Function will find *.fst if not provided.
        '''

        # Define OpenFAST input filename
        if fastfile:
            print('Using {} to run OpenFAST simulation'.format(fastfile))
        else:
            for file in os.listdir(fast_dir):
                if file.endswith('.fst'):
                    fastfile = file
                    print(file)

        # save starting file path -- note: This is an artifact of needing to call OpenFAST from the same directory as DISCON.IN
        original_path = os.getcwd()
        # change path, run sim
        os.chdir(fast_dir)
        os.system('{} {}'.format(fastcall, os.path.join(fast_dir,'*.fst')))
        # return to original path
        os.chdir(original_path)

    def plot_fast_out(self):
        '''
        Plot OpenFAST outputs 
            - NJA: this is a good place to emulate Post_LoadFastOut.m
        '''

    def read_fast_out(self):
        '''
        Read OpenFAST output files. Might want to leverage AeroelasticSE here.
        '''

class FileProcessing():
    """
    Class FileProcessing used to write out controller 
        parameter files need to run ROSCO

    Methods:
    -----------
    write_param_file
    write_rotor_performance
    """

    def __init__(self):
        pass
    def write_param_file(self, turbine, controller, param_file='DISCON.IN', txt_filename='Cp_Ct_Cq.txt'):
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
        print('Writing new controller parameter file parameter file: %s.' % param_file)
        # Should be obvious what's going on here...
        file = open(param_file,'w')
        file.write('! Controller parameter input file for the %s wind turbine\n' % turbine.TurbineName)
        file.write('!    - File written using NREL Reference Controller tuning logic on %s\n' % now.strftime('%m/%d/%y'))
        file.write('\n')
        file.write('!------- DEBUG ------------------------------------------------------------\n')
        file.write('{0:<12d}        ! LoggingLevel		- {{0: write no debug files, 1: write standard output .dbg-file, 2: write standard output .dbg-file and complete avrSWAP-array .dbg2-file}}\n'.format(controller.LoggingLevel))
        file.write('\n')
        file.write('!------- CONTROLLER FLAGS -------------------------------------------------\n')
        file.write('{0:<12d}        ! F_LPFType			- {{1: first-order low-pass filter, 2: second-order low-pass filter}}, [rad/s] (currently filters generator speed and pitch control signals\n'.format(controller.F_LPFType))
        file.write('{0:<12d}        ! F_NotchType		- Notch on the measured generator speed {{0: disable, 1: enable}}\n'.format(controller.F_NotchType))
        file.write('{0:<12d}        ! IPC_ControlMode	- Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {{0: off, 1: 1P reductions, 2: 1P+2P reductions}}\n'.format(controller.IPC_ControlMode))
        file.write('{0:<12d}        ! VS_ControlMode	- Generator torque control mode in above rated conditions {{0: constant torque, 1: constant power, 2: TSR tracking PI control}}\n'.format(controller.VS_ControlMode))
        file.write('{0:<12d}        ! PC_ControlMode    - Blade pitch control mode {{0: No pitch, fix to fine pitch, 1: active PI blade pitch control}}\n'.format(controller.PC_ControlMode))
        file.write('{0:<12d}        ! Y_ControlMode		- Yaw control mode {{0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC}}\n'.format(controller.Y_ControlMode))
        file.write('{0:<12d}        ! SS_Mode           - Setpoint Smoother mode {{0: no setpoint smoothing, 1: introduce setpoint smoothing}}\n'.format(controller.SS_Mode))
        file.write('{0:<12d}        ! WE_Mode           - Wind speed estimator mode {{0: One-second low pass filtered hub height wind speed, 1: Immersion and Invariance Estimator (Ortega et al.)}}\n'.format(controller.WE_Mode))
        file.write('{0:<12d}        ! PS_Mode           - Peak shaving mode {{0: no peak shaving, 1: implement peak shaving}}\n'.format(controller.PS_Mode))
        file.write('\n')
        file.write('!------- FILTERS ----------------------------------------------------------\n') 
        file.write('{:<13.2f}       ! F_LPFCornerFreq	- Corner frequency (-3dB point) in the low-pass filters, [rad/s]\n'.format(turbine.bld_edgewise_freq * 1/4)) 
        file.write('{:<13.2f}       ! F_LPFDamping		- Damping coefficient [used only when F_FilterType = 2]\n'.format(0.0))
        file.write('{:<13.2f}       ! F_NotchCornerFreq	- Natural frequency of the notch filter, [rad/s]\n'.format(0.0))
        file.write('{:<6.1f}{:<13.1f} ! F_NotchBetaNumDen	- Two notch damping values (numerator and denominator, resp) - determines the width and depth of the notch, [-]\n'.format(0.0,0.0))
        file.write('{:<014.6f}      ! F_SSCornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother, [rad/s].\n'.format(controller.ss_cornerfreq))
        file.write('\n')
        file.write('!------- BLADE PITCH CONTROL ----------------------------------------------\n')
        file.write('{:<11d}         ! PC_GS_n			- Amount of gain-scheduling table entries\n'.format(len(controller.pitch_op_pc)))
        file.write('{}              ! PC_GS_angles	    - Gain-schedule table: pitch angles\n'.format(''.join('{:<4.6f}  '.format(controller.pitch_op_pc[i]) for i in range(len(controller.pitch_op_pc)))))            
        file.write('{}              ! PC_GS_KP		- Gain-schedule table: pitch controller kp gains\n'.format(''.join('{:<4.6f}  '.format(controller.pc_gain_schedule.Kp[i]) for i in range(len(controller.pc_gain_schedule.Kp)))))
        file.write('{}              ! PC_GS_KI		- Gain-schedule table: pitch controller ki gains\n'.format(''.join('{:<4.6f}  '.format(controller.pc_gain_schedule.Ki[i]) for i in range(len(controller.pc_gain_schedule.Ki)))))
        file.write('{}              ! PC_GS_KD			- Gain-schedule table: pitch controller kd gains\n'.format(''.join('{:<1.1f}  '.format(0.0) for i in range(len(controller.pc_gain_schedule.Ki)))))
        file.write('{}              ! PC_GS_TF			- Gain-schedule table: pitch controller tf gains (derivative filter)\n'.format(''.join('{:<1.1f}  '.format(0.0) for i in range(len(controller.pc_gain_schedule.Ki)))))
        file.write('{:<014.5f}      ! PC_MaxPit			- Maximum physical pitch limit, [rad].\n'.format(controller.max_pitch))
        file.write('{:<014.5f}      ! PC_MinPit			- Minimum physical pitch limit, [rad].\n'.format(controller.min_pitch))
        file.write('{:<014.5f}      ! PC_MaxRat			- Maximum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(turbine.max_pitch_rate))
        file.write('{:<014.5f}      ! PC_MinRat			- Minimum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(turbine.min_pitch_rate))
        file.write('{:<014.5f}      ! PC_RefSpd			- Desired (reference) HSS speed for pitch controller, [rad/s].\n'.format(turbine.rated_rotor_speed*turbine.Ng))
        file.write('{:<014.5f}      ! PC_FinePit		- Record 5: Below-rated pitch angle set-point, [rad]\n'.format(controller.min_pitch))
        file.write('{:<014.5f}      ! PC_Switch			- Angle above lowest minimum pitch angle for switch, [rad]\n'.format(1 * deg2rad))
        file.write('{:<11d}         ! Z_EnableSine		- Enable/disable sine pitch excitation, used to validate for dynamic induction control, will be removed later, [-]\n'.format(0))
        file.write('{:<014.5f}      ! Z_PitchAmplitude	- Amplitude of sine pitch excitation, [rad]\n'.format(0.0))
        file.write('{:<014.5f}      ! Z_PitchFrequency	- Frequency of sine pitch excitation, [rad/s]\n'.format(0.0))
        file.write('\n')
        file.write('!------- INDIVIDUAL PITCH CONTROL -----------------------------------------\n')
        file.write('{:<13.1f}       ! IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from IPC), [rad]\n'.format(0.0))
        file.write('{:<6.1f}{:<13.1f} ! IPC_KI			- Integral gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n'.format(0.0,0.0))
        file.write('{:<6.1f}{:<13.1f} ! IPC_aziOffset		- Phase offset added to the azimuth angle for the individual pitch controller, [rad]. \n'.format(0.0,0.0))
        file.write('{:<13.1f}       ! IPC_CornerFreqAct - Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {{0: Disable}}, [rad/s]\n'.format(0.0))
        file.write('\n')
        file.write('!------- VS TORQUE CONTROL ------------------------------------------------\n')
        file.write('{:<014.5f}      ! VS_GenEff			- Generator efficiency mechanical power -> electrical power, [should match the efficiency defined in the generator properties!], [-]\n'.format(turbine.GenEff))
        file.write('{:<014.5f}      ! VS_ArSatTq		- Above rated generator torque PI control saturation, [Nm]\n'.format(turbine.rated_torque))
        file.write('{:<014.5f}      ! VS_MaxRat			- Maximum torque rate (in absolute value) in torque controller, [Nm/s].\n'.format(turbine.max_torque_rate))
        file.write('{:<014.5f}      ! VS_MaxTq			- Maximum generator torque in Region 3 (HSS side), [Nm].\n'.format(turbine.rated_torque*1.1))
        file.write('{:<014.5f}      ! VS_MinTq			- Minimum generator (HSS side), [Nm].\n'.format(0.0))
        file.write('{:<014.5f}      ! VS_MinOMSpd		- Optimal mode minimum speed, cut-in speed towards optimal mode gain path, [rad/s]\n'.format(controller.vs_minspd))
        file.write('{:<014.5f}      ! VS_Rgn2K			- Generator torque constant in Region 2 (HSS side), [N-m/(rad/s)^2]\n'.format(controller.vs_rgn2K))
        file.write('{:<014.5f}      ! VS_RtPwr			- Wind turbine rated power [W]\n'.format(turbine.rated_power))
        file.write('{:<014.5f}      ! VS_RtTq			- Rated torque, [Nm].\n'.format(turbine.rated_torque))
        file.write('{:<014.5f}      ! VS_RefSpd			- Rated generator speed [rad/s]\n'.format(controller.vs_refspd))
        file.write('{:<11d}         ! VS_n				- Number of generator PI torque controller gains\n'.format(1))
        file.write('{:<014.5f}      ! VS_KP				- Proportional gain for generator PI torque controller [1/(rad/s) Nm]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(controller.vs_gain_schedule.Kp[-1]))
        file.write('{:<014.5f}      ! VS_KI				- Integral gain for generator PI torque controller [1/rad Nm]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(controller.vs_gain_schedule.Ki[-1]))
        file.write('{:<13.2f}       ! VS_TSRopt			- Power-maximizing region 2 tip-speed-ratio [rad].\n'.format(turbine.Cp.TSR_opt))
        file.write('\n')
        file.write('!------- SETPOINT SMOOTHER ---------------------------------------------\n')
        file.write('{:<13.5f}       ! SS_VSGain         - Variable speed torque controller setpoint smoother gain, [-].\n'.format(controller.ss_vsgain))
        file.write('{:<13.5f}       ! SS_PCGain         - Collective pitch controller setpoint smoother gain, [-].\n'.format(controller.ss_pcgain))
        file.write('\n')
        file.write('!------- WIND SPEED ESTIMATOR ---------------------------------------------\n')
        file.write('{:<13.3f}       ! WE_BladeRadius	- Blade length [m]\n'.format(turbine.rotor_radius))
        file.write('{:<11d}         ! WE_CP_n			- Amount of parameters in the Cp array\n'.format(1))
        file.write(          '{}    ! WE_CP - Parameters that define the parameterized CP(lambda) function\n'.format(''.join('{:<2.1f} '.format(0.0) for i in range(4))))
        file.write('{:<13.1f}		! WE_Gamma			- Adaption gain of the wind speed estimator algorithm [m/rad]\n'.format(0.0))
        file.write('{:<13.1f}       ! WE_GearboxRatio	- Gearbox ratio [>=1],  [-]\n'.format(turbine.Ng))
        file.write('{:<014.5f}      ! WE_Jtot			- Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, [kg m^2]\n'.format(turbine.J))
        file.write('{:<13.3f}       ! WE_RhoAir			- Air density, [kg m^-3]\n'.format(turbine.rho))
        file.write(      '"{}"      ! PerfFileName      - File containing rotor performance tables (Cp,Ct,Cq)\n'.format(turbine.rotor_performance_filename))
        file.write('{:<7d} {:<10d}  ! PerfTableSize     - Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios\n'.format(len(turbine.Cp.pitch_initial_rad),len(turbine.Cp.TSR_initial)))
        file.write('{:<11d}         ! WE_FOPoles_N      - Number of first-order system poles used in EKF\n'.format(len(controller.A)))
        file.write('{}              ! WE_FOPoles_v      - Wind speeds corresponding to first-order system poles [m/s]\n'.format(''.join('{:<4.2f} '.format(controller.v[i]) for i in range(len(controller.v)))))
        file.write('{}              ! WE_FOPoles        - First order system poles\n'.format(''.join('{:<10.8f} '.format(controller.A[i]) for i in range(len(controller.A)))))
        file.write('\n')
        file.write('!------- YAW CONTROL ------------------------------------------------------\n')
        file.write('{:<13.1f}       ! Y_ErrThresh		- Yaw error threshold. Turbine begins to yaw when it passes this. [rad^2 s]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]\n'.format(0.0))
        file.write('{:<11d}         ! Y_IPC_n			- Number of controller gains (yaw-by-IPC)\n'.format(1))
        file.write('{:<13.1f}       ! Y_IPC_KP			- Yaw-by-IPC proportional controller gain Kp\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_KI			- Yaw-by-IPC integral controller gain Ki\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_omegaLP		- Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_zetaLP		- Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_MErrSet			- Yaw alignment error, set point [rad]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_omegaLPFast		- Corner frequency fast low pass filter, 1.0 [Hz]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_omegaLPSlow		- Corner frequency slow low pass filter, 1/60 [Hz]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_Rate			- Yaw rate [rad/s]\n'.format(0.0))
        file.write('\n')
        file.write('!------- TOWER FORE-AFT DAMPING -------------------------------------------\n')
        file.write('{:<11d}         ! FA_KI				- Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m] - !NJA - Make this a flag\n'.format(-1))
        file.write('{:<13.1f}       ! FA_HPF_CornerFreq	- Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]\n'.format(0.0))
        file.write('{:<13.1f}       ! FA_IntSat			- Integrator saturation (maximum signal amplitude contribution to pitch from FA damper), [rad]\n'.format(0.0))
        file.write('\n')
        file.write('!------- PEAK SHAVING -------------------------------------------\n')
        file.write('{:<11d}         ! PS_BldPitchMin_N  - Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)\n'.format(len(controller.ps.pitch_min)))
        file.write('{}              ! PS_WindSpeeds       - Wind speeds corresponding to minimum blade pitch angles [m/s]\n'.format(''.join('{:<4.2f} '.format(controller.ps.v[i]) for i in range(len(controller.ps.v)))))
        file.write('{}              ! PS_BldPitchMin          - Minimum blade pitch angles [rad]'.format(''.join('{:<10.8f} '.format(controller.ps.pitch_min[i]) for i in range(len(controller.ps.pitch_min)))))
        file.close()

    def write_rotor_performance(self,turbine,txt_filename='Cp_Ct_Cq.txt'):
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
        file.write('# Pitch angle vector - x axis (matrix columns) (deg)\n')
        for i in range(len(turbine.Cp.pitch_initial_rad)):
            file.write('{:0.4}   '.format(turbine.Cp.pitch_initial_rad[i] * rad2deg))
        file.write('\n# TSR vector - y axis (matrix rows) (-)\n')
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