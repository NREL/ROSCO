# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import numpy as np
from ccblade import CCAirfoil, CCBlade
from scipy import interpolate, gradient

from WTC_toolbox import turbine as wtc_turbine

turbine = wtc_turbine.Turbine()

# Some useful constants
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)

class Controller():
    """
    Class controller can be used to read in / write out controller param files
    And update tunings
    """

    def __init__(self):
        """
        Maybe just initialize the internal variables
        This also lists what will need to be defined
        """
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


    def write_param_file(self, param_file):
        """
        Load the parameter files directly from a FAST input deck
        """
    
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
        A = np.empty(len(TSR_op))
        B_beta = np.empty(len(TSR_op))
        dCp_dbeta = np.empty(len(TSR_op))
        dCp_dTSR = np.empty(len(TSR_op))
        # ------------- Find Linearized State Matrices ------------- #

        for i in range(len(TSR_op)):

            # Find pitch angle as a function of expected operating CP for each TSR
            self.Cp_TSR = np.ndarray.flatten(turbine.Cp.interp_surface(turbine.pitch_initial_rad, TSR_op[i]))     # all Cp values for a given tsr
            Cp_op[i] = np.clip(Cp_op[i], np.min(self.Cp_TSR), np.max(self.Cp_TSR))                                      # saturate Cp values to be on Cp surface
            f_cp_pitch = interpolate.interp1d(self.Cp_TSR,pitch_initial_rad)                                # interpolate function for Cp(tsr) values
            pitch_op[i] = f_cp_pitch(Cp_op[i])                                                              # expected operation blade pitch values
            
            dCp_dbeta[i], dCp_dTSR[i] = turbine.Cp.interp_gradient(pitch_op[i],TSR_op[i])
        
        dCp_dbeta = dCp_dbeta/np.diff(pitch_initial_rad)[0]
        dCp_dTSR = dCp_dTSR/np.diff(TSR_initial)[0]
        
        # Linearized system derivative
        dtau_dbeta = Ng/2*rho*Ar*R*(1/TSR_op)*dCp_dbeta*v**2
        dtau_dlambda = Ng/2*rho*Ar*R*v**2*(1/(TSR_op**2))*(dCp_dTSR*TSR_op - Cp_op)
        dlambda_domega = R/v/Ng
        dtau_domega = dtau_dlambda*dlambda_domega

        # Second order system coefficiencts
        A = dtau_domega/J             # Plant pole
        B_tau = -Ng**2/J              # Torque input gain 
        B_beta = dtau_dbeta/J         # Blade pitch input gain

        # Wind Disturbance Input
        dlambda_dv = -(TSR_op/v)
        dtau_dv = dtau_dlambda*dlambda_dv
        B_v = dtau_dv/J


        # separate and define below and above rated parameters
        A_vs = A[0:len(v_below_rated)]          # below rated
        A_pc = A[len(v_below_rated):len(v)]     # above rated
        B_tau = B_tau * np.ones(len(v_below_rated))
        B_beta = B_beta[len(v_below_rated):len(v)]

        # Find gain schedule
        self.pc_gain_schedule = GainSchedule()
        self.pc_gain_schedule.second_order_PI(zeta_pc, omega_pc,A_pc,B_beta,linearize=True,v=v_above_rated)
        self.vs_gain_schedule = GainSchedule()
        self.vs_gain_schedule.second_order_PI(zeta_vs, omega_vs,A_vs,B_tau,linearize=False,v=v_below_rated)

        # Store some variables
        self.v = v          # Wind speed (m/s)
        self.Cp_op = Cp_op
        self.pitch_op = pitch_op
        self.pitc_op_pc = pitch_op[len(v_below_rated):len(v)]
        self.TSR_op = TSR_op
        self.A = A 
        self.B_beta = B_beta

class GainSchedule():
    def __init__(self):
        '''
        Gain Schedule class used to define gain schedules for desired closed loop dynamics
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


    