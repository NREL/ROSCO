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
        self.PC_zeta = param_file.PC_zeta            # Pitch controller damping ratio (-)
        self.PC_omega = param_file.PC_omega                # Pitch controller natural frequency (rad/s)
        
        # Torque Controller Parameters
        self.VS_zeta = param_file.VS_zeta            # Torque controller damping ratio (-)
        self.VS_omega = param_file.VS_omega                # Torque controller natural frequency (rad/s)
        
        # Setpoint Smoother Parameters
        self.Kss_PC = param_file.Kss_PC              # Pitch controller reference gain bias 
        self.Kss_VS = param_file.Kss_VS              # Torque controller reference gain bias
        self.Vmin = turbine.VS_Vmin                  # Cut-in wind speed (m/s)
        self.Vrat = turbine.PC_Vrated                # Rated wind speed (m/s)
        self.Vmax = turbine.PC_Vmax                  # Cut-out wind speed (m/s), -- Does not need to be exact


    def write_param_file(self, param_file):
        """
        Load the parameter files directly from a FAST input deck
        """

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
        PC_zeta = self.PC_zeta                  # Pitch controller damping ratio
        PC_omega = self.PC_omega                # Pitch controller natural frequency (rad/s)
        VS_zeta = self.VS_zeta                  # Torque controller damping ratio (-)
        VS_omega = self.VS_omega                # Torque controller natural frequency (rad/s)
        Vrat = self.Vrat                        # Rated wind speed (m/s)
        Vmin = turbine.Vmin                     # Cut in wind speed (m/s)
        Vmax = turbine.Vmax                     # Cut out wind speed (m/s)
 
        # -------------Define Operation Points ------------- #
        TSR_rated = RRspeed*R/Vrat  # TSR at rated

        # separate wind speeds by operation regions
        v_br = np.arange(Vmin,Vrat,0.1)             # below rated
        v_ar = np.arange(Vrat,Vmax,0.1)             # above rated
        v = np.concatenate((v_br, v_ar))

        # separate TSRs by operations regions
        TSR_br = np.ones(len(v_br))*turbine.Cp.TSR_opt # below rated     
        TSR_ar = RRspeed*R/v_ar                     # above rated
        TSR_op = np.concatenate((TSR_br, TSR_ar))   # operational TSRs

        # Find expected operational Cp values
        Cp_above_rated = turbine.Cp.interp_surface(0,TSR_ar[0])             # Cp during rated operation (not optimal). Assumes cut-in bld pitch to be 0
        Cp_op_br = np.ones(len(v_br)) * turbine.Cp.max              # below rated
        Cp_op_ar = Cp_above_rated * (TSR_ar/TSR_rated)**3           # above rated
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

        # System state "matrices"
        A = dtau_domega/J             # Plant pole
        B_tau = -Ng**2/J              # Torque input gain 
        B_beta = dtau_dbeta/J         # Blade pitch input gain

        # Wind Disturbance Input
        dlambda_dv = -TSR_op/v
        dtau_dv = dtau_dlambda*dlambda_dv
        B_v = dtau_dv/J

        # Store some variables
        self.v = v          # Wind speed (m/s)
        self.Cp_op = Cp_op
        self.pitch_op = pitch_op
        self.TSR_op = TSR_op
        self.A = A 
        self.B_beta = B_beta

# Beta_del = Betavec(2) - Betavec(1);
# TSR_del = TSRvec(2) - TSRvec(1);
            
    def controller_params(self):
    # Hard coded controller parameters for turbine. Using this until read_param_file is good to go
    #           - Coded for NREL 5MW 

        # Pitch Controller Parameters
        self.PC_zeta = 0.7                      # Pitch controller damping ratio (-)
        self.PC_omega = 0.6                     # Pitch controller natural frequency (rad/s)
        
        # Torque Controller Parameters
        self.VS_zeta = 0.7                      # Torque controller damping ratio (-)
        self.VS_omega = 0.3                     # Torque controller natural frequency (rad/s)
        
        # Other basic parameters
        self.Vrat = 11.4                        # Rated wind speed (m/s)
