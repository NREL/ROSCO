# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import numpy as np
import datetime
from ccblade import CCAirfoil, CCBlade
from scipy import interpolate, gradient
from WTC_toolbox import turbine as wtc_turbine

now = datetime.datetime.now()
turbine = wtc_turbine.Turbine()

# Some useful constants
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)

class Controller():
    """
    Class controller used to calculate controller tunings parameters
    """

    def __init__(self):
        pass
    
    def controller_params(self):
    # Hard coded controller parameters for turbine. Using this until read_param_file is good to go
    #           - Coded for NREL 5MW 

        # Pitch Controller Parameters
        self.zeta_pc = 0.7                      # Pitch controller damping ratio (-)
        self.omega_pc = 0.6                     # Pitch controller natural frequency (rad/s)
        
        # Torque Controller Parameters
        self.zeta_vs = 0.7                      # Torque controller damping ratio (-)
        self.omega_vs = 0.3                     # Torque controller natural frequency (rad/s)
        
        # Other basic parameters
        self.v_rated = 11.4                        # Rated wind speed (m/s)

    def tune_controller(self, turbine):
        """
        Given a turbine model, tune the controller parameters
        """
        # -------------Load Parameters ------------- #
        # Re-define Turbine Parameters for shorthand
        J = turbine.J                           # Total rotor inertial (kg-m^2) 
        rho = turbine.rho                       # Air density (kg/m^3)
        R = turbine.RotorRad                    # Rotor radius (m)
        Ar = np.pi*R**2                         # Rotor area (m^2)
        Ng = turbine.Ng                         # Gearbox ratio (-)
        RRspeed = turbine.RRspeed               # Rated rotor speed (rad/s)

        # Load controller parameters 
        #   - should be self.read_param_file() eventually, hard coded for now
        self.controller_params()

        # Re-define controller tuning parameters for shorthand
        zeta_pc = self.zeta_pc                  # Pitch controller damping ratio
        omega_pc = self.omega_pc                # Pitch controller natural frequency (rad/s)
        zeta_vs = self.zeta_vs                  # Torque controller damping ratio (-)
        omega_vs = self.omega_vs                # Torque controller natural frequency (rad/s)
        v_rated = self.v_rated                        # Rated wind speed (m/s)
        v_min = turbine.v_min                     # Cut in wind speed (m/s)
        v_max = turbine.v_max                     # Cut out wind speed (m/s)
 
        # -------------Define Operation Points ------------- #
        TSR_rated = RRspeed*R/v_rated  # TSR at rated

        # separate wind speeds by operation regions
        v_below_rated = np.arange(v_min,v_rated,0.1)             # below rated
        v_above_rated = np.arange(v_rated,v_max,0.1)             # above rated
        v = np.concatenate((v_below_rated, v_above_rated))

        # separate TSRs by operations regions
        TSR_below_rated = np.ones(len(v_below_rated))*turbine.Cp.TSR_opt # below rated     
        TSR_above_rated = RRspeed*R/v_above_rated                     # above rated
        TSR_op = np.concatenate((TSR_below_rated, TSR_above_rated))   # operational TSRs

        # Find expected operational Cp values
        Cp_above_rated = turbine.Cp.interp_surface(0,TSR_above_rated[0])             # Cp during rated operation (not optimal). Assumes cut-in bld pitch to be 0
        Cp_op_br = np.ones(len(v_below_rated)) * turbine.Cp.max              # below rated
        Cp_op_ar = Cp_above_rated * (TSR_above_rated/TSR_rated)**3           # above rated
        Cp_op = np.concatenate((Cp_op_br, Cp_op_ar))                # operational CPs to linearize around
        pitch_initial_rad = turbine.pitch_initial_rad
        TSR_initial = turbine.TSR_initial

        # initialize variables
        pitch_op = np.empty(len(TSR_op))
        dCp_beta = np.empty(len(TSR_op))
        dCp_TSR = np.empty(len(TSR_op))
        # ------------- Find Linearized State Matrices ------------- #

        for i in range(len(TSR_op)):

            # Find pitch angle as a function of expected operating CP for each TSR
            self.Cp_TSR = np.ndarray.flatten(turbine.Cp.interp_surface(turbine.pitch_initial_rad, TSR_op[i]))     # all Cp values for a given tsr
            Cp_op[i] = np.clip(Cp_op[i], np.min(self.Cp_TSR), np.max(self.Cp_TSR))      # saturate Cp values to be on Cp surface
            f_cp_pitch = interpolate.interp1d(self.Cp_TSR,pitch_initial_rad)        # interpolate function for Cp(tsr) values
            pitch_op[i] = f_cp_pitch(Cp_op[i])      # expected operation blade pitch values
            dCp_beta[i], dCp_TSR[i] = turbine.Cp.interp_gradient(pitch_op[i],TSR_op[i])       # gradients of Cp surface in Beta and TSR directions
        
        # Full Cp surface gradients
        dCp_dbeta = dCp_beta/np.diff(pitch_initial_rad)[0]
        dCp_dTSR = dCp_TSR/np.diff(TSR_initial)[0]
        
        # Linearized system derivatives
        dtau_dbeta = Ng/2*rho*Ar*R*(1/TSR_op)*dCp_dbeta*v**2
        dtau_dlambda = Ng/2*rho*Ar*R*v**2*(1/(TSR_op**2))*(dCp_dTSR*TSR_op - Cp_op)
        dlambda_domega = R/v/Ng
        dtau_domega = dtau_dlambda*dlambda_domega

        # Second order system coefficiencts
        A = dtau_domega/J             # Plant pole
        B_tau = -Ng**2/J              # Torque input  
        B_beta = dtau_dbeta/J         # Blade pitch input 

        # Wind Disturbance Input
        dlambda_dv = -(TSR_op/v)
        dtau_dv = dtau_dlambda*dlambda_dv
        B_v = dtau_dv/J # wind speed input - currently unused 


        # separate and define below and above rated parameters
        A_vs = A[0:len(v_below_rated)]          # below rated
        A_pc = A[len(v_below_rated):len(v)]     # above rated
        B_tau = B_tau * np.ones(len(v_below_rated))
        B_beta = B_beta[len(v_below_rated):len(v)]

        # Find gain schedule
        self.pc_gain_schedule = ControllerTypes()
        self.pc_gain_schedule.second_order_PI(zeta_pc, omega_pc,A_pc,B_beta,linearize=True,v=v_above_rated)
        self.vs_gain_schedule = ControllerTypes()
        self.vs_gain_schedule.second_order_PI(zeta_vs, omega_vs,A_vs,B_tau,linearize=False,v=v_below_rated)

        # Store some variables
        self.v = v                                  # Wind speed (m/s)
        self.v_below_rated = v_below_rated
        self.Cp_op = Cp_op
        self.pitch_op = pitch_op
        self.pitch_op_pc = pitch_op[len(v_below_rated):len(v)]
        self.TSR_op = TSR_op
        self.A = A 
        self.B_beta = B_beta

class ControllerTypes():
    def __init__(self):
        '''
        Controller Types class used to define gains for desired closed loop dynamics
        '''
        pass

    def second_order_PI(self,zeta,om_n,A,B,linearize=False,v=None):

        # Linearize system coefficients w.r.t. wind speed if desired
        if linearize:
            print('Calculating second order PI gain schedule for linearized system pole location.')
            pA = np.polyfit(v,A,1)
            pB = np.polyfit(v,B,1)
            A = pA[0]*v + pA[1]
            B = pB[0]*v + pB[1]

        # Calculate gain schedule
        self.Kp = 1/B * (2*zeta*om_n + A)
        self.Ki = om_n**2/B           

class FileProcessing():
    """
    Class ProcessFile can be used to read in / write out controller parameter files to update
    """

    def __init__(self):
        pass
    def read_param_file(self, param_file):
        """
        Load the parameter files directly from a FAST input deck
        """
        # Pitch Controller Parameters
        self.zeta_pc = param_file.PC_zeta            # Pitch controller damping ratio (-)
        self.omega_pc = param_file.PC_omega                # Pitch controller natural frequency (rad/s)
        
        # Torque Controller Parameters
        self.zeta_vs = param_file.VS_zeta            # Torque controller damping ratio (-)
        self.omega_vs = param_file.VS_omega                # Torque controller natural frequency (rad/s)
        
        # Setpoint Smoother Parameters
        self.Kss_PC = param_file.Kss_PC              # Pitch controller reference gain bias 
        self.Kss_VS = param_file.Kss_VS              # Torque controller reference gain bias
        self.v_min = turbine.VS_Vmin                  # Cut-in wind speed (m/s)
        self.v_rated = turbine.PC_Vrated                # Rated wind speed (m/s)
        self.v_max = turbine.PC_Vmax                  # Cut-out wind speed (m/s), -- Does not need to be exact


    def write_param_file(self, param_file, turbine, controller, new_file=True):
        """
        Print the controller parameters to the DISCON.IN input file for the generic controller

        Parameters:
        -----------
            param_file: str
                        filename to for parameter input file
            turbine: class
                     class containing 
            new_file: bool
                      True = create new file, False = modify existing file.  (False functionality not included yet)
        """
        if new_file:
            print('Writing new controller parameter file parameter file %s.' % param_file)
            file = open(param_file,'w')
            file.write('! Controller parameter input file for the %s wind turbine\n' % turbine.TurbineName)
            file.write('!    - File written using NREL Baseline Controller tuning logic on %s\n' % now.strftime('%m/%d/%y'))
            file.write('\n')
            file.write('!------- DEBUG ------------------------------------------------------------\n')
            file.write('1					! LoggingLevel		- {0: write no debug files, 1: write standard output .dbg-file, 2: write standard output .dbg-file and complete avrSWAP-array .dbg2-file\n')
            file.write('\n')
            file.write('!------- CONTROLLER FLAGS -------------------------------------------------\n')
            file.write('1					! F_LPFType			- {1: first-order low-pass filter, 2: second-order low-pass filter}, [rad/s] (currently filters generator speed and pitch control signals)\n')
            file.write('0					! F_NotchType		- Notch on the measured generator speed {0: disable, 1: enable} \n')
            file.write('0					! IPC_ControlMode	- Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {0: off, 1: 1P reductions, 2: 1P+2P reductions}\n')
            file.write('2					! VS_ControlMode	- Generator torque control mode in above rated conditions {0: constant torque, 1: constant power, 2: TSR tracking PI control}\n')
            file.write('1                   ! PC_ControlMode    - Blade pitch control mode {0: No pitch, fix to fine pitch, 1: active PI blade pitch control}\n')
            file.write('0					! Y_ControlMode		- Yaw control mode {0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC}\n')
            file.write('1                   ! SS_Mode           - Setpoint Smoother mode {0: no setpoint smoothing, 1: introduce setpoint smoothing}\n')
            file.write('2                   ! WE_Mode           - Wind speed estimator mode {0: One-second low pass filtered hub height wind speed, 1: Imersion and Invariance Estimator (Ortega et al.)}\n')
            file.write('0                   ! PS_Mode           - Peak shaving mode {0: no peak shaving, 1: implement peak shaving}\n')
            file.write('\n')
            file.write('!------- FILTERS ----------------------------------------------------------\n') 
            file.write('1.570796326			! F_LPFCornerFreq	- Corner frequency (-3dB point) in the low-pass filters, [Hz]\n') # this needs to be included as an input file
            file.write('0					! F_LPFDamping		- Damping coefficient [used only when F_FilterType = 2]\n')
            file.write('0					! F_NotchCornerFreq	- Natural frequency of the notch filter, [rad/s]\n')
            file.write('0	0				! F_NotchBetaNumDen	- Two notch damping values (numerator and denominator, resp) - determines the width and depth of the notch, [-]\n')
            file.write('0.5                 ! F_SSCornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother, [Hz].\n')
            file.write('\n')
            file.write('!------- BLADE PITCH CONTROL ----------------------------------------------\n')
            file.write('{}                  ! PC_GS_n			- Amount of gain-scheduling table entries\n'.format(len(controller.pitch_op_pc)))
            file.write('{}                  ! PC_GS_angles	    - Gain-schedule table: pitch angles\n'.format(str(controller.pitch_op_pc).strip('[]').replace('\n',''))) 
            file.write('{}                  ! PC_GS_KP		- Gain-schedule table: pitch controller kp gains\n'.format(str(controller.pc_gain_schedule.Kp).strip('[]').replace('\n','')))
            file.write('{}                  ! PC_GS_KI		- Gain-schedule table: pitch controller ki gains\n'.format(str(controller.pc_gain_schedule.Ki).strip('[]').replace('\n','')))
            file.write('{}                   ! PC_GS_KD			- Gain-schedule table: pitch controller kd gains\n'.format(str(np.zeros(len(controller.pitch_op_pc))).strip('[]').replace('\n','')))
            file.write('{}                   ! PC_GS_TF			- Gain-schedule table: pitch controller tf gains (derivative filter)\n'.format(str(np.zeros(len(controller.pitch_op_pc))).strip('[]').replace('\n','')))
            file.write('1.5707				! PC_MaxPit			- Maximum physical pitch limit, [rad].\n')
            file.write('-0.087266			! PC_MinPit			- Minimum physical pitch limit, [rad].\n')
            file.write('0.13962				! PC_MaxRat			- Maximum pitch rate (in absolute value) in pitch controller, [rad/s].\n')
            file.write('-0.13962			! PC_MinRat			- Minimum pitch rate (in absolute value) in pitch controller, [rad/s].\n')
            file.write('122.90957			! PC_RefSpd			- Desired (reference) HSS speed for pitch controller, [rad/s].\n')
            file.write('0.0					! PC_FinePit		- Record 5: Below-rated pitch angle set-point, [rad]\n')
            file.write('0.003490658			! PC_Switch			- Angle above lowest minimum pitch angle for switch, [rad]\n')
            file.write('0					! Z_EnableSine		- Enable/disable sine pitch excitation, used to validate for dynamic induction control, will be removed later, [-]\n')
            file.write('0.0349066			! Z_PitchAmplitude	- Amplitude of sine pitch excitation, [rad]\n')
            file.write('0					! Z_PitchFrequency	- Frequency of sine pitch excitation, [rad/s]\n')
            file.write('\n')
            file.write('!------- INDIVIDUAL PITCH CONTROL -----------------------------------------\n')
            file.write('0.087266			! IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from IPC), [rad]\n')
            file.write('1E-8 0				! IPC_KI			- Integral gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n')
            file.write('0.436332313	0		! IPC_aziOffset		- Phase offset added to the azimuth angle for the individual pitch controller, [rad]. \n')
            file.write('2.5					! IPC_CornerFreqAct - Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {0: Disable}, [rad/s]\n')
            file.write('\n')
            file.write('!------- VS TORQUE CONTROL ------------------------------------------------\n')
            file.write('0.944				! VS_GenEff			- Generator efficiency mechanical power -> electrical power, [should match the efficiency defined in the generator properties!], [-]\n')
            file.write('43093.55			! VS_ArSatTq		- Above rated generator torque PI control saturation, [Nm]\n')
            file.write('150000.0			! VS_MaxRat			- Maximum torque rate (in absolute value) in torque controller, [Nm/s].\n')
            file.write('48000.00			! VS_MaxTq			- Maximum generator torque in Region 3 (HSS side), [Nm].\n')
            file.write('0.0					! VS_MinTq			- Minimum generator (HSS side), [Nm].\n')
            file.write('91.2109				! VS_MinOMSpd		- Optimal mode minimum speed, cut-in speed towards optimal mode gain path, [rad/s]\n')
            file.write('2.33228				! VS_Rgn2K			- Generator torque constant in Region 2 (HSS side), [N-m/(rad/s)^2]\n')
            file.write('5.0E+06				! VS_RtPwr			- Wind turbine rated power [W]\n')
            file.write('43093.55			! VS_RtTq			- Rated torque, [Nm].\n')
            file.write('120.113				! VS_RefSpd			- Rated generator speed [rad/s]\n')
            file.write('1					! VS_n				- Number of generator PI torque controller gains\n')
            file.write('-835				! VS_KP				- Proportional gain for generator PI torque controller [1/(rad/s) Nm]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n')
            file.write('-164	 			! VS_KI				- Integral gain for generator PI torque controller [1/rad Nm]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n')
            file.write('\n')
            file.write('!------- SETPOINT SMOOTHER ---------------------------------------------\n')
            file.write('30                  ! SS_VSGainBias     - Variable speed torque controller gain bias, [(rad/s)/rad].\n')
            file.write('0.001               ! SS_PCGainBias     - Collective pitch controller gain bias, [(rad/s)/Nm].\n')
            file.write('\n')
            file.write('!------- WIND SPEED ESTIMATOR ---------------------------------------------\n')
            file.write('63.0				! WE_BladeRadius	- Blade length [m]\n')
            file.write('4					! WE_CP_n			- Amount of parameters in the Cp array\n')
            file.write('14.571319658214513	42.809556250371465	2.456512501523107	0.003127994078720	! WE_CP - Parameters that define the parameterized CP(lambda) function\n')
            file.write('20					! WE_Gamma			- Adaption gain of the wind speed estimator algorithm [m/rad]\n')
            file.write('97					! WE_GearboxRatio	- Gearbox ratio [>=1],  [-]\n')
            file.write('4.0469564E+07		! WE_Jtot			- Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, [kg m^2]\n')
            file.write('1.225				! WE_RhoAir			- Air density, [kg m^-3]\n')
            file.write('"../5MW_Baseline/Cp_Ct_Cq.txt"      ! PerfFileName      - File containing rotor performance tables (Cp,Ct,Cq)\n')
            file.write('320  40             ! PerfTableSize     - Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios\n')
            file.write('23                  ! WE_FOPoles_N      - Number of first-order system poles used in EKF\n')
            file.write('3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25   ! WE_FOPoles_v      - Wind speeds corresponding to first-order system poles [m/s]\n')
            file.write('-0.0203 -0.0270 -0.0338 -0.0405 -0.0473 -0.0540 -0.0608 -0.0675 -0.0743 -0.0671 -0.0939 -0.1257 -0.1601 -0.1973 -0.2364 -0.2783 -0.3223 -0.3678 -0.4153 -0.4632 -0.5122 -0.5629 -0.6194   ! WE_FOPoles        - First order system poles\n')
            file.write('\n')
            file.write('!------- YAW CONTROL ------------------------------------------------------\n')
            file.write('1.745329252			! Y_ErrThresh		- Yaw error threshold. Turbine begins to yaw when it passes this. [rad^2 s]\n')
            file.write('0.17453				! Y_IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]\n')
            file.write('1					! Y_IPC_n			- Number of controller gains (yaw-by-IPC)\n')
            file.write('-0.064				! Y_IPC_KP			- Yaw-by-IPC proportional controller gain Kp\n')
            file.write('-0.0008				! Y_IPC_KI			- Yaw-by-IPC integral controller gain Ki\n')
            file.write('0.6283185			! Y_IPC_omegaLP		- Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].\n')
            file.write('1.0					! Y_IPC_zetaLP		- Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].\n')
            file.write('0.00000				! Y_MErrSet			- Yaw alignment error, set point [rad]\n')
            file.write('1.0					! Y_omegaLPFast		- Corner frequency fast low pass filter, 1.0 [Hz]\n')
            file.write('0.016667			! Y_omegaLPSlow		- Corner frequency slow low pass filter, 1/60 [Hz]\n')
            file.write('0.0034906			! Y_Rate			- Yaw rate [rad/s]\n')
            file.write('\n')
            file.write('!------- TOWER FORE-AFT DAMPING -------------------------------------------\n')
            file.write('-1					! FA_KI				- Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m] - !NJA - Make this a flag\n')
            file.write('0.1                 ! FA_HPF_CornerFreq	- Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]\n')
            file.write('0.087266			! FA_IntSat			- Integrator saturation (maximum signal amplitude contribution to pitch from FA damper), [rad]\n')
            file.write('\n')
            file.write('!------- PEAK SHAVING -------------------------------------------\n')
            file.write('23                  ! PS_BldPitchMin_N  - Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)\n')
            file.write('3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 ! PS_WindSpeeds       - Wind speeds corresponding to minimum blade pitch angles [m/s]\n')
            file.write(' -0.0349 -0.0349 -0.0349 -0.0349 -0.0349 -0.0349 -0.0349 -0.0114 0.0356 0.0577 0.0825 0.1058 0.1282 0.1499 0.1708 0.1913 0.2114 0.2310 0.2502 0.2690 0.2874 0.3056 0.3239 ! PS_BldPitchMin          - Minimum blade pitch angles [rad]\n')
            file.close()
