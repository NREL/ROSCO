# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import numpy as numpy
from ccblade import CCAirfoil, CCBlade

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
        self.PC_om = param_file.PC_om                # Pitch controller natural frequency (rad/s)
        
        # Torque Controller Parameters
        self.VS_zeta = param_file.VS_zeta            # Torque controller damping ratio (-)
        self.VS_om = param_file.VS_om                # Torque controller natural frequency (rad/s)
        
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

        # Turbine Parameters
        J = turbine.J                           # Total rotor inertial (kg-m^2) 
        rho = turbine.rho                       # Air density (kg/m^3)
        R = turbine.RotorRad                    # Rotor radius (m)
        Ar = pi*R^2                             # Rotor area (m^2)
        Ng = turbine.GBRatio                    # Gearbox ratio (-)
        RRspeed = turbine.RRSpeed               # Rated rotor speed (rad/s)
        
        # Cp Surface
        CpSurf = turbine.CpSurf                 # Matrix of Cp surface values
        CpBeta = turbine.CpBeta                 # Vector of blade pitch angles corresponding to Cp surface (rad)
        CpTSR = turbine.CpTSR                   # Vector of tip-speed-ratio values corresponding to Cp surface (rad)
