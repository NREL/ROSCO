# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import os
import numpy as np
from ccblade import CCAirfoil, CCBlade
from AeroelasticSE.FAST_reader import InputReader_OpenFAST
from scipy import interpolate
import pickle

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


        # Turbine Parameters
        self.J = None                   # Total rotor inertial (kg-m^2) 
        self.rho = None                      # Air density (kg/m^3)
        self.RotorRad = None                    # Rotor radius (m)
        self.Ng = None # Gearbox ratio (-)
        self.RRspeed = None               # Rated rotor speed (rad/s)
        self.Vmin = 4.                  # Cut-in wind speed (m/s) (JUST ASSUME FOR NOW)
        self.Vrat = None                # Rated wind speed (m/s)
        self.Vmax = 25.                  # Cut-out wind speed (m/s), -- Does not need to be exact (JUST ASSUME FOR NOW)

        # Init the cc-blade rotor
        self.cc_rotor = None

        # Cp Surface
        # self.CpSurf = None                 # Matrix of Cp surface values
        # self.CpBeta = None                 # Vector of blade pitch angles corresponding to Cp surface (rad)
        # self.CpTSR = None                   # Vector of tip-speed-ratio values corresponding to Cp surface (rad)
        
        # Interp function versions
        self.cp_interp = None
        self.ct_interp = None
        self.cq_interp = None

    # Allow print out of class
    def __str__(self): 

        print('---------------------')
        print('Turbine Info')
        print('J: %.1f' % self.J)
        print('rho: %.1f' % self.rho)
        print('RotorRad: %.1f' % self.RotorRad)
        print('---------------------')
        return ' '

    # Save function
    def save(self,filename):
        tuple_to_save = (self.J,self.rho,self.RotorRad, self.Ng,self.RRspeed,self.Vmin,self.Vrat,self.Vmax,self.cc_rotor,self.cp_interp,self.ct_interp,self.cq_interp   )
        pickle.dump( tuple_to_save, open( filename, "wb" ) )

    # Load function
    def load(self, filename):
        self.J,self.rho,self.RotorRad, self.Ng,self.RRspeed,self.Vmin,self.Vrat,self.Vmax,self.cc_rotor,self.cp_interp,self.ct_interp,self.cq_interp = pickle.load(open(filename,'rb'))


    def load_from_fast(self, FAST_InputFile,FAST_directory,drivetrain_inertia, FAST_ver='OpenFAST',dev_branch=True,rot_source=None, txt_filename=None):
        """
        Load the parameter files directly from a FAST input deck
        Inputs:
            Fast_InputFile - Primary fast model input file (*.fst)
            FAST_directory - Directory for primary fast model input file
            drivetrain_intertia - drivetrain intertia (kg-m^2)                      # nja - this might be able to be automated 
            dev_branch - dev_branch input to InputReader_OpenFAST, probably True
            rot_source - desired source for rotor to get Cp, Ct, Cq tables. Default is to run cc-blade. 
                            options: cc-blade - run cc-blade
                                     txt - from *.txt file
            txt_filename - filename for *.txt, only used if rot_source='txt'
        """

        # Need unfortunately to hack this for now, hope to fix later
        self.J = drivetrain_inertia 

        print('Loading FAST model: %s ' % FAST_InputFile)
        fast = InputReader_OpenFAST(FAST_ver=FAST_ver,dev_branch=dev_branch)
        fast.FAST_InputFile = FAST_InputFile
        fast.FAST_directory = FAST_directory
        fast.execute()


        # Grab general turbine parameters
        TipRad = fast.fst_vt['ElastoDyn']['TipRad']
        Rhub =  fast.fst_vt['ElastoDyn']['HubRad']
        hubHt = 90. #HARD CODED UNTIL FIND A SOLUTION TO READ OUTPUT
        shearExp = 0.2  #HARD CODED UNTIL FIND A SOLUTION TO READ OUTPUT
        rho = fast.fst_vt['AeroDyn15']['AirDens']
        mu = fast.fst_vt['AeroDyn15']['KinVisc']

        # Store values needed by controller 
        self.Ng = fast.fst_vt['ElastoDyn']['GBRatio']
        self.rho = rho
        self.RotorRad = TipRad

        # Calculate rated rotor speed for now by scaling from NREL 5MW
        self.RRspeed = (63. / TipRad) * 12.1 * rpmRadSec

        # Load Cp, Ct, Cq,  surfaces
        if rot_source == 'txt':
            self.load_from_txt(fast,txt_filename)
        elif rot_source == 'cc-blade':
            self.load_from_ccblade(fast)
        else:   # default load from cc-blade
            print('No desired rotor performance data source specified, running cc-blade by default.')
            self.load_from_ccblade(fast)

    def load_from_ccblade(self,fast):
        print('Loading rotor performace data from cc-blade:')
        # Create CC-Blade Rotor
        r = np.array(fast.fst_vt['AeroDynBlade']['BlSpn'])
        theta = np.array(fast.fst_vt['AeroDynBlade']['BlTwist'])
        chord = np.array(fast.fst_vt['AeroDynBlade']['BlChord'])
        af_idx = np.array(fast.fst_vt['AeroDynBlade']['BlAFID']).astype(int) - 1 #Reset to 0 index
        AFNames = fast.fst_vt['AeroDyn15']['AFNames']

        # NEED A PRETTY SERIOUS HACK TO POINT AT OLD AIRFOIL TABLES
        ad14path = '/Users/pfleming/Desktop/git_tools/FLORIS/examples/5MW_AFFiles'
        AFNames_14 = []
        for airfoil in AFNames:
            a_name = os.path.basename(airfoil)
            AFNames_14.append(os.path.join(ad14path,a_name))

        # Load in the airfoils
        afinit = CCAirfoil.initFromAerodynFile  # just for shorthand
        airfoil_types = []
        for airfoil in AFNames_14:
            airfoil_types.append(afinit(airfoil))

        af = [0]*len(r)
        for i in range(len(r)):
            af[i] = airfoil_types[af_idx[i]]

        tilt = fast.fst_vt['ElastoDyn']['ShftTilt'] 
        precone = fast.fst_vt['ElastoDyn']['PreCone1']
        yaw = 0.0

        nSector = 8  # azimuthal discretization

        B = 3 #fixed at 3-bladed for now

        # Now save the CC-Blade rotor
        self.cc_rotor = CCBlade(r, chord, theta, af, Rhub, TipRad, B, rho, mu,
                        precone, tilt, yaw, shearExp, hubHt, nSector)


        # Generate the look-up tables
        # Mesh the grid and flatten the arrays
        fixed_rpm = 10 # RPM

        TSR_initial = np.arange(0.5,15,0.5)
        pitch_initial = np.arange(0,25,0.5)
        ws_array = (fixed_rpm * (np.pi / 30.) * TipRad)  / TSR_initial
        ws_mesh, pitch_mesh = np.meshgrid(ws_array, pitch_initial)
        ws_flat = ws_mesh.flatten()
        pitch_flat = pitch_mesh.flatten()
        omega_flat = np.ones_like(pitch_flat) * fixed_rpm
        tsr_flat = (fixed_rpm * (np.pi / 30.) * TipRad)  / ws_flat

        # Get values from cc-blade
        P, T, Q, M, CP, CT, CQ, CM = self.cc_rotor.evaluate(ws_flat, omega_flat, pitch_flat, coefficients=True)

        # Reshape Cp, Ct and Cq
        CP = np.reshape(CP, (len(pitch_initial), len(TSR_initial)))
        CT = np.reshape(CT, (len(pitch_initial), len(TSR_initial)))
        CQ = np.reshape(CQ, (len(pitch_initial), len(TSR_initial)))

        pitch_initial_rad = pitch_initial * pi/180
        # # Form the interpolant functions which can look up any arbitrary location
        self.cp_interp = interpolate.interp2d(pitch_initial_rad, TSR_initial, np.transpose(CP), kind='cubic')
        self.ct_interp = interpolate.interp2d(pitch_initial_rad, TSR_initial, np.transpose(CT), kind='cubic')
        self.cq_interp = interpolate.interp2d(pitch_initial_rad, TSR_initial, np.transpose(CQ), kind='cubic')

    def load_from_txt(self,fast,txt_filename):
        print('Loading rotor performace data from text file:', txt_filename)
        
        # Assign variables
        CP = []
        CT = []
        CQ = []

        with open(txt_filename) as pfile:
            for line in pfile:
                # Read Blade Pitch Angles (degrees)
                if line.__contains__('Pitch angle'):
                    # Beta.append(pfile.readline())
                    pitch_initial = np.array([float(x) for x in pfile.__next__().strip().split()])
                    pitch_initial = pitch_initial * (np.pi/180) # degrees to rad
                # Read Tip Speed Ratios (rad)
                if line.__contains__('TSR'):
                    TSR_initial = np.array([float(x) for x in pfile.__next__().strip().split()])
                
                # Read Power Coefficients
                if line.__contains__('Power'):
                    pfile.__next__()
                    for tsr_i in range(len(TSR_initial)):
                        CP = np.append(CP,np.array([float(x) for x in pfile.__next__().strip().split()]))
                    CP = np.reshape(CP, (len(TSR_initial),len(pitch_initial)))
                
                # Read Thrust Coefficients
                if line.__contains__('Thrust'):
                    pfile.__next__()
                    for tsr_i in range(len(TSR_initial)):
                        CT = np.append(CT,np.array([float(x) for x in pfile.__next__().strip().split()]))
                    CT = np.reshape(CT, (len(TSR_initial),len(pitch_initial)))

                # Read Troque Coefficients
                if line.__contains__('Torque'):
                    pfile.__next__()
                    for tsr_i in range(len(TSR_initial)):
                        CQ = np.append(CQ,np.array([float(x) for x in pfile.__next__().strip().split()]))
                    CQ = np.reshape(CQ, (len(TSR_initial),len(pitch_initial)))
        
        # # Form the interpolant functions which can look up any arbitrary location
        self.cp_interp = interpolate.interp2d(pitch_initial, TSR_initial, CP, kind='cubic',bounds_error=False)
        self.ct_interp = interpolate.interp2d(pitch_initial, TSR_initial, CT, kind='cubic',bounds_error=False))
        self.cq_interp = interpolate.interp2d(pitch_initial, TSR_initial, CQ, kind='cubic',bounds_error=False))


    # # NOT CERTAIN OF THESE ALTERNATIVES YET
    # def load_from_sowfa(self, fast_folder):
    #     """
    #     Load the parameter files directly from a SOWFA directory
    #     """

    # def load_from_csv(self, fast_folder):
    #     """
    #     Load from a simple CSV file containing the parameters
    #     """