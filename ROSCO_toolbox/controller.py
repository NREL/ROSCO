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
from wisdem.ccblade import CCAirfoil, CCBlade
from scipy import interpolate, gradient

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class Controller():
    """
    Class Controller used to calculate controller tunings parameters


    Methods:
    -------
    tune_controller

    Parameters:
    -----------
    controller_params: dict
                       Dictionary containing controller paramaters that need to be defined
    """

    def __init__(self, controller_params):
        ''' 
        Load controller tuning parameters from input dictionary
        '''

        print('-----------------------------------------------------------------------------')
        print('   Tuning a reference wind turbine controller using NREL\'s ROSCO toolbox    ')
        print('      Developed by Nikhar J. Abbas for collaborotave research purposes.      ')
        print('-----------------------------------------------------------------------------')

        # Controller Flags
        self.LoggingLevel = controller_params['LoggingLevel']
        self.F_LPFType = controller_params['F_LPFType']
        self.F_NotchType = controller_params['F_NotchType']
        self.IPC_ControlMode = controller_params['IPC_ControlMode']
        self.VS_ControlMode = controller_params['VS_ControlMode']
        self.PC_ControlMode = controller_params['PC_ControlMode']
        self.Y_ControlMode = controller_params['Y_ControlMode']
        self.SS_Mode = controller_params['SS_Mode']
        self.WE_Mode = controller_params['WE_Mode']
        self.PS_Mode = controller_params['PS_Mode']
        self.SD_Mode = controller_params['SD_Mode']
        self.Fl_Mode = controller_params['Fl_Mode']

        # Necessary parameters
        self.zeta_pc = controller_params['zeta_pc']
        self.omega_pc = controller_params['omega_pc']
        self.zeta_vs = controller_params['zeta_vs']
        self.omega_vs = controller_params['omega_vs']

        # Optional parameters, default to standard if not defined
        if controller_params['min_pitch']:
            self.min_pitch = controller_params['min_pitch']
        else:
            self.min_pitch = None
        
        if controller_params['max_pitch']:
            self.max_pitch = controller_params['max_pitch']
        else:
            self.max_pitch = 90*deg2rad      # Default to 90 degrees max pitch
        
        if controller_params['vs_minspd']:
            self.vs_minspd = controller_params['vs_minspd']
        else:
            self.vs_minspd = None 

        if controller_params['ss_vsgain']:
            self.ss_vsgain = controller_params['ss_vsgain']
        else:
            self.ss_vsgain = 1.      # Default to 100% setpoint shift
        
        if controller_params['ss_pcgain']:
            self.ss_pcgain = controller_params['ss_pcgain']
        else:
            self.ss_pcgain = 0.001      # Default to 0.1% setpoint shift
        
        if controller_params['ss_cornerfreq']:
            self.ss_cornerfreq = controller_params['ss_cornerfreq']
        else:
            self.ss_cornerfreq = .62831850001     # Default to 10 second time constant 
        
        if controller_params['ps_percent']:
            self.ps_percent = controller_params['ps_percent']
        else:
            self.ps_percent = 0.75      # Default to 75% peak shaving

        # critical damping if LPFType = 2
        if controller_params['F_LPFType']:
            if controller_params['F_LPFType'] == 2:
                self.F_LPFDamping = 0.7
            else:
                self.F_LPFDamping = 0.0
        else:
            self.F_LPFDamping = 0.0

        # Shutdown filter default cornering freq at 15s time constant
        if controller_params['sd_cornerfreq']:
            self.sd_cornerfreq = controller_params['sd_cornerfreq']
        else:
            self.sd_cornerfreq = 0.41888
        
        if controller_params['sd_maxpit']:
            self.sd_maxpit = controller_params['sd_maxpit']
        else:
            self.sd_maxpit = None

    def tune_controller(self, turbine):
        """
        Given a turbine model, tune a controller based on the NREL generic controller tuning process

        Parameters:
        -----------
        turbine : class
                  Turbine class containing necessary turbine information to accurately tune the controller. 
        """
        # -------------Load Parameters ------------- #
        # Re-define Turbine Parameters for shorthand
        J = turbine.J                           # Total rotor inertial (kg-m^2) 
        rho = turbine.rho                       # Air density (kg/m^3)
        R = turbine.rotor_radius                    # Rotor radius (m)
        Ar = np.pi*R**2                         # Rotor area (m^2)
        Ng = turbine.Ng                         # Gearbox ratio (-)
        rated_rotor_speed = turbine.rated_rotor_speed               # Rated rotor speed (rad/s)


        # -------------Define Operation Points ------------- #
        TSR_rated = rated_rotor_speed*R/turbine.v_rated  # TSR at rated

        # separate wind speeds by operation regions
        v_below_rated = np.arange(turbine.v_min,turbine.v_rated,0.5)             # below rated
        v_above_rated = np.arange(turbine.v_rated+0.5,turbine.v_max,0.5)             # above rated
        v = np.concatenate((v_below_rated, v_above_rated))

        # separate TSRs by operations regions
        TSR_below_rated = np.ones(len(v_below_rated))*turbine.Cp.TSR_opt # below rated     
        TSR_above_rated = rated_rotor_speed*R/v_above_rated                     # above rated
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

        # ------------- Find Linearized State "Matrices" ------------- #
        for i in range(len(TSR_op)):
            # Find pitch angle as a function of expected operating CP for each TSR
            Cp_TSR = np.ndarray.flatten(turbine.Cp.interp_surface(turbine.pitch_initial_rad, TSR_op[i]))     # all Cp values for a given tsr
            Cp_op[i] = np.clip(Cp_op[i], np.min(Cp_TSR), np.max(Cp_TSR))        # saturate Cp values to be on Cp surface
            f_cp_pitch = interpolate.interp1d(Cp_TSR,pitch_initial_rad)         # interpolate function for Cp(tsr) values
            pitch_op[i] = f_cp_pitch(Cp_op[i])                                  # expected operation blade pitch values
            dCp_beta[i], dCp_TSR[i] = turbine.Cp.interp_gradient(pitch_op[i],TSR_op[i])       # gradients of Cp surface in Beta and TSR directions
        
        # Define minimum pitch saturation to be at Cp-maximizing pitch angle if not specifically defined
        if not self.min_pitch:
            self.min_pitch = pitch_op[0]

        # Full Cp surface gradients
        dCp_dbeta = dCp_beta/np.diff(pitch_initial_rad)[0]
        dCp_dTSR = dCp_TSR/np.diff(TSR_initial)[0]
        
        # Linearized system derivatives
        dtau_dbeta = Ng/2*rho*Ar*R*(1/TSR_op)*dCp_dbeta*v**2
        dtau_dlambda = Ng/2*rho*Ar*R*v**2*(1/(TSR_op**2))*(dCp_dTSR*TSR_op - Cp_op)
        dlambda_domega = R/v/Ng
        dtau_domega = dtau_dlambda*dlambda_domega

        # Second order system coefficients
        A = dtau_domega/J             # Plant pole
        B_tau = -Ng**2/J              # Torque input  
        B_beta = dtau_dbeta/J         # Blade pitch input 

        # Wind Disturbance Input
        dlambda_dv = -(TSR_op/v)
        dtau_dv = dtau_dlambda*dlambda_dv
        # B_v = dtau_dv/J # wind speed input - currently unused 


        # separate and define below and above rated parameters
        A_vs = A[0:len(v_below_rated)]          # below rated
        A_pc = A[len(v_below_rated):len(v)]     # above rated
        B_tau = B_tau * np.ones(len(v_below_rated))
        B_beta = B_beta[len(v_below_rated):len(v)]

        # -- Find gain schedule --
        self.pc_gain_schedule = ControllerTypes()
        self.pc_gain_schedule.second_order_PI(self.zeta_pc, self.omega_pc,A_pc,B_beta,linearize=True,v=v_above_rated)
        self.vs_gain_schedule = ControllerTypes()
        self.vs_gain_schedule.second_order_PI(self.zeta_vs, self.omega_vs,A_vs,B_tau,linearize=False,v=v_below_rated)

        # -- Find K for Komega_g^2 --
        self.vs_rgn2K = (pi*rho*R**5.0 * turbine.Cp.max) / (2.0 * turbine.Cp.TSR_opt**3 * Ng**3)
        self.vs_refspd = min(turbine.Cp.TSR_opt * turbine.v_rated/R, turbine.rated_rotor_speed) * Ng

        # -- Define some setpoints --
        # minimum rotor speed saturation limits
        if self.vs_minspd:
            self.vs_minspd = np.maximum(self.vs_minspd, (turbine.Cp.TSR_opt * turbine.v_min / turbine.rotor_radius) * Ng)
        else: 
            self.vs_minspd = (turbine.Cp.TSR_opt * turbine.v_min / turbine.rotor_radius) * Ng
        self.pc_minspd = self.vs_minspd

        # max pitch angle for shutdown
        if self.sd_maxpit:
            self.sd_maxpit = self.sd_maxpit
        else:
            self.sd_maxpit = pitch_op[-1]

        # Store some variables
        self.v = v                                  # Wind speed (m/s)
        self.v_below_rated = v_below_rated
        self.pitch_op = pitch_op
        self.pitch_op_pc = pitch_op[len(v_below_rated):len(v)]
        self.TSR_op = TSR_op
        self.A = A 
        self.B_beta = B_beta
        self.B_tau = B_tau
        # - Might want these to debug -
        # self.Cp_op = Cp_op

        # --- Minimum pitch saturation ---
        self.ps_min_bld_pitch = np.ones(len(self.pitch_op)) * self.min_pitch
        self.ps = ControllerBlocks()

        if self.PS_Mode == 1:  # Peak Shaving
            self.ps.peak_shaving(self, turbine)
        elif self.PS_Mode == 2: # Cp-maximizing minimum pitch saturation
            self.ps.min_pitch_saturation(self,turbine)
        elif self.PS_Mode == 3: # Peak shaving and Cp-maximizing minimum pitch saturation
            self.ps.peak_shaving(self, turbine)
            self.ps.min_pitch_saturation(self,turbine)

        # --- Floating feedback term ---
        if self.Fl_Mode == 1: # Floating feedback
            Kpf = (dtau_dv/dtau_dbeta)*turbine.TowerHt * Ng;
            self.Kpf = Kpf[-1]
        else:
            self.Kpf = 0.0
        
class ControllerBlocks():
    '''
    Class ControllerBlocks defines tuning parameters for additional controller features or "blocks"

    Methods:
    --------
    peak_shaving

    '''
    def __init__(self):
        pass
    
    def peak_shaving(self,controller, turbine):
        ''' 
        Define minimum blade pitch angle for peak shaving routine based on a maximum allowable thrust 

        Parameters:
        -----------
        controller: class
                    Controller class containing controller operational information
        turbine: class
                 Turbine class containing necessary wind turbine information for controller tuning
        '''

        # Re-define Turbine Parameters for shorthand
        J = turbine.J                           # Total rotor inertial (kg-m^2) 
        rho = turbine.rho                       # Air density (kg/m^3)
        R = turbine.rotor_radius                    # Rotor radius (m)
        A = np.pi*R**2                         # Rotor area (m^2)
        Ng = turbine.Ng                         # Gearbox ratio (-)
        rated_rotor_speed = turbine.rated_rotor_speed               # Rated rotor speed (rad/s)

        # Initialize some arrays
        Ct_op = np.empty(len(controller.TSR_op),dtype='float64')
        Ct_max = np.empty(len(controller.TSR_op),dtype='float64')
        beta_min = np.empty(len(controller.TSR_op),dtype='float64')
        # Find unshaved rotor thurst coefficients and associated rotor thrusts
        # for i in len(controller.TSR_op):
        for i in range(len(controller.TSR_op)):
            Ct_op[i] = turbine.Ct.interp_surface(controller.pitch_op[i],controller.TSR_op[i])
            T = 0.5 * rho * A * controller.v**2 * Ct_op

        # Define minimum max thrust and initialize pitch_min
        Tmax = controller.ps_percent * np.max(T)
        pitch_min = np.ones(len(controller.pitch_op)) * controller.min_pitch

        # Modify pitch_min if max thrust exceeds limits
        for i in range(len(controller.TSR_op)):
            # Find Ct values for operational TSR
            # Ct_tsr = turbine.Ct.interp_surface(turbine.pitch_initial_rad, controller.TSR_op[i])
            Ct_tsr = turbine.Ct.interp_surface(turbine.pitch_initial_rad,controller.TSR_op[i])
            # Define max Ct values
            Ct_max[i] = Tmax/(0.5 * rho * A * controller.v[i]**2)
            if T[i] > Tmax:
                Ct_op[i] = Ct_max[i]
            else:
                Ct_max[i] = np.minimum( np.max(Ct_tsr), Ct_max[i])
            # Define minimum pitch angle
            f_pitch_min = interpolate.interp1d(Ct_tsr, turbine.pitch_initial_rad, bounds_error=False, fill_value=(turbine.pitch_initial_rad[0],turbine.pitch_initial_rad[-1]))
            pitch_min[i] = f_pitch_min(Ct_max[i])

        controller.ps_min_bld_pitch = pitch_min

        # save some outputs for analysis or future work
        self.Tshaved = 0.5 * rho * A * controller.v**2 * Ct_op
        self.pitch_min = pitch_min
        self.v = controller.v
        self.Ct_max = Ct_max
        self.Ct_op = Ct_op
        self.T = T

    def min_pitch_saturation(self, controller, turbine):
        
        # Find TSR associated with minimum rotor speed
        TSR_at_minspeed = (controller.pc_minspd/turbine.Ng) * turbine.rotor_radius / controller.v_below_rated
        for i in range(len(TSR_at_minspeed)):
            if TSR_at_minspeed[i] > controller.TSR_op[i]:
                controller.TSR_op[i] = TSR_at_minspeed[i]
        
                # Initialize some arrays
                Cp_op = np.empty(len(turbine.pitch_initial_rad),dtype='float64')
                min_pitch = np.empty(len(TSR_at_minspeed),dtype='float64')
                
        
                # Find Cp-maximizing minimum pitch schedule
                # for j in range(len(TSR_at_minspeed)):
                # Find Cp coefficients at below-rated tip speed ratios
                Cp_op = turbine.Cp.interp_surface(turbine.pitch_initial_rad,TSR_at_minspeed[i])
                Cp_max = max(Cp_op)
                f_pitch_min = interpolate.interp1d(Cp_op, turbine.pitch_initial_rad, bounds_error=False, fill_value=(turbine.pitch_initial_rad[0],turbine.pitch_initial_rad[-1]))
                min_pitch[i] = f_pitch_min(Cp_max)
                
                # modify existing minimum pitch schedule
                # for pind, pitch in enumerate(min_itch):
                controller.ps_min_bld_pitch[i] = np.maximum(controller.ps_min_bld_pitch[i], min_pitch[i])
            else:
                return


class ControllerTypes():
    '''
    Class ControllerTypes used to define any types of controllers that can be tuned. 
        Generally, calculates gains based on some pre-defined tuning parameters. 

    Methods:
    --------
    second_order_PI
    '''
    def __init__(self):
        pass

    def second_order_PI(self,zeta,om_n,A,B,linearize=False,v=None):
        '''
        Define proportional integral gain schedule for a closed
            loop system with a standard second-order form.

        Parameters:
        -----------
        zeta : int (-)
               Desired damping ratio 
        om_n : int (rad/s)
               Desired natural frequency 
        A : array_like (1/s)
            Plant poles (state transition matrix)
        B : array_like (varies)
            Plant numerators (input matrix)
        linearize : bool, optional
                    If 'True', find a gain scheduled based on a linearized plant.
        v : array_like (m/s)
            Wind speeds for linearized plant model, if desired. 
        '''
        # Linearize system coefficients w.r.t. wind speed if desired
        if linearize:
            pA = np.polyfit(v,A,1)
            pB = np.polyfit(v,B,1)
            A = pA[0]*v + pA[1]
            B = pB[0]*v + pB[1]

        # Calculate gain schedule
        self.Kp = 1/B * (2*zeta*om_n + A)
        self.Ki = om_n**2/B           