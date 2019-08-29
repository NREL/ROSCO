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
from AeroelasticSE.FAST_reader import InputReader_OpenFAST

# Some useful constants
degRad = np.pi/180.
rpmRadSec = 2.0*(np.pi)/60.0

class Turbine():
    """
    Class turbine defines a turbine in terms of what is needed to design the controller
    and to run the 'tiny' simulation
    """

    def __init__(self):
        """
        Maybe just initialize the internal variables
        This also lists what will need to be defined
        """
        # Should names match fast or can be more simple
        self.gb_ratio = None # Initialize all to none?

    def load_from_fast(self, FAST_InputFile,FAST_directory, FAST_ver='OpenFAST',dev_branch=True):
        """
        Load the parameter files directly from a FAST input deck
        """
        print('Loading FAST model: %s ' % FAST_InputFile)
        fast = InputReader_OpenFAST(FAST_ver=FAST_ver,dev_branch=dev_branch)
        fast.FAST_InputFile = FAST_InputFile
        fast.FAST_directory = FAST_directory
        fast.execute()

        # Now grab any values the controller might need
        self.gb_ratio = fast.fst_vt['ElastoDyn']['GBRatio']
        print(self.gb_ratio)

        # Define the CCBLADE rotor
        TipRad = fast.fst_vt['Fst7']['TipRad']
        Rhub =  fast.fst_vt['Fst7']['HubRad']
        hubHt = 90. #HARD CODED UNTIL FIND A SOLUTION TO READ OUTPUT
        shearExp = 0.2  #HARD CODED UNTIL FIND A SOLUTION TO READ OUTPUT
        rho = 1.225 
        mu = 1.81206e-5

        # Get the along the blade arrays and check them
        r = fast.fst_vt['AeroDynBlade']['RNodes']
        print(r)
        print(np.array([2.8667, 5.6000, 8.3333, 11.7500, 15.8500, 19.9500, 24.0500,
                    28.1500, 32.2500, 36.3500, 40.4500, 44.5500, 48.6500, 52.7500,
                    56.1667, 58.9000, 61.6333]))
        print(fast.fst_vt['AeroDyn15']['NumAFfiles'])


# Function returns a scaled NREL 5MW rotor object from CC-Blade
# path_to_af='5MW_AFFiles'):

    # chord = (Rtip/base_R)*np.array([3.542, 3.854, 4.167, 4.557, 4.652, 4.458, 4.249, 4.007, 3.748,
    #                     3.502, 3.256, 3.010, 2.764, 2.518, 2.313, 2.086, 1.419])
    # theta = np.array([13.308, 13.308, 13.308, 13.308, 11.480, 10.162, 9.011, 7.795,
    #                     6.544, 5.361, 4.188, 3.125, 2.319, 1.526, 0.863, 0.370, 0.106])
    # B = 3  # number of blades

    # # In this initial version, hard-code to be NREL 5MW
    # afinit = CCAirfoil.initFromAerodynFile  # just for shorthand
    # basepath = path_to_af

    # # load all airfoils
    # airfoil_types = [0]*8
    # airfoil_types[0] = afinit(path.join(basepath, 'Cylinder1.dat'))
    # airfoil_types[1] = afinit(path.join(basepath, 'Cylinder2.dat'))
    # airfoil_types[2] = afinit(path.join(basepath, 'DU40_A17.dat'))
    # airfoil_types[3] = afinit(path.join(basepath, 'DU35_A17.dat'))
    # airfoil_types[4] = afinit(path.join(basepath, 'DU30_A17.dat'))
    # airfoil_types[5] = afinit(path.join(basepath, 'DU25_A17.dat'))
    # airfoil_types[6] = afinit(path.join(basepath, 'DU21_A17.dat'))
    # airfoil_types[7] = afinit(path.join(basepath, 'NACA64_A17.dat'))

    # # place at appropriate radial stations
    # af_idx = [0, 0, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 7]

    # af = [0]*len(r)
    # for i in range(len(r)):
    #     af[i] = airfoil_types[af_idx[i]]


    # tilt = -5.0
    # precone = 2.5
    # yaw = 0.0

    # nSector = 8  # azimuthal discretization

    # rotor = CCBlade(r, chord, theta, af, Rhub, Rtip, B, rho, mu,
    #                 precone, tilt, yaw, shearExp, hubHt, nSector)

    # return rotor


    # # NOT CERTAIN OF THESE ALTERNATIVES YET
    # def load_from_sowfa(self, fast_folder):
    #     """
    #     Load the parameter files directly from a SOWFA directory
    #     """

    # def load_from_csv(self, fast_folder):
    #     """
    #     Load from a simple CSV file containing the parameters
    #     """