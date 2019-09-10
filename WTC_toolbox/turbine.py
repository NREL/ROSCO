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
from scipy import interpolate, gradient
import pickle

# Some useful constants
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0

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
        self.v_min = 4.                  # Cut-in wind speed (m/s) (JUST ASSUME FOR NOW)
        self.v_rated = None                # Rated wind speed (m/s)
        self.v_max = 25.                  # Cut-out wind speed (m/s), -- Does not need to be exact (JUST ASSUME FOR NOW)

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
        tuple_to_save = (self.J,self.rho,self.RotorRad, self.Ng,self.RRspeed,self.v_min,self.v_rated,self.v_max,self.cc_rotor,self.cp_interp,self.ct_interp,self.cq_interp   )
        pickle.dump( tuple_to_save, open( filename, "wb" ) )

    # Load function
    def load(self, filename):
        self.J,self.rho,self.RotorRad, self.Ng,self.RRspeed,self.v_min,self.v_rated,self.v_max,self.cc_rotor,self.cp_interp,self.ct_interp,self.cq_interp = pickle.load(open(filename,'rb'))


    def load_from_fast(self, FAST_InputFile,FAST_directory,drivetrain_inertia, FAST_ver='OpenFAST',dev_branch=True,rot_source=None, txt_filename=None):
        """
        Load the parameter files directly from a FAST input deck

        Parameters:
        -----------
            Fast_InputFile: str
                            Primary fast model input file (*.fst)
            FAST_directory: str
                            Directory for primary fast model input file
            drivetrain_intertia: int
                                drivetrain intertia (kg-m^2)                # nja - this might be able to be automated 
            FAST_ver: string, optional

            dev_branch: bool
                        dev_branch input to InputReader_OpenFAST, probably True
            rot_source: str
                        desired source for rotor to get Cp, Ct, Cq tables. Default is to run cc-blade. 
                            options: cc-blade - run cc-blade
                                     txt - from *.txt file
            txt_filename: str
                          filename for *.txt, only used if rot_source='txt'
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
        self.RRspeed = (63. / TipRad) * 12.1 * rpm2RadSec

        # Load Cp, Ct, Cq tables
        if rot_source == 'txt':
            self.load_from_txt(fast,txt_filename)
        elif rot_source == 'cc-blade':
            self.load_from_ccblade(fast)
        else:   # default load from cc-blade
            print('No desired rotor performance data source specified, running cc-blade by default.')
            self.load_from_ccblade(fast)

        # Parse rotor performance data
        self.Cp = RotorPerformance(self.Cp_table,self.pitch_initial_rad,self.TSR_initial)
        self.Ct = RotorPerformance(self.Ct_table,self.pitch_initial_rad,self.TSR_initial)
        self.Cq = RotorPerformance(self.Cq_table,self.pitch_initial_rad,self.TSR_initial)

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
        pitch_initial_rad = pitch_initial * deg2rad
        ws_array = (fixed_rpm * (np.pi / 30.) * TipRad)  / TSR_initial
        ws_mesh, pitch_mesh = np.meshgrid(ws_array, pitch_initial)
        ws_flat = ws_mesh.flatten()
        pitch_flat = pitch_mesh.flatten()
        omega_flat = np.ones_like(pitch_flat) * fixed_rpm
        tsr_flat = (fixed_rpm * (np.pi / 30.) * TipRad)  / ws_flat

        # Get values from cc-blade
        P, T, Q, M, CP, CT, CQ, CM = self.cc_rotor.evaluate(ws_flat, omega_flat, pitch_flat, coefficients=True)

        # Reshape Cp, Ct and Cq
        CP = np.transpose(np.reshape(CP, (len(pitch_initial), len(TSR_initial))))
        CT = np.transpose(np.reshape(CT, (len(pitch_initial), len(TSR_initial))))
        CQ = np.transpose(np.reshape(CQ, (len(pitch_initial), len(TSR_initial))))

        # Store necessary metrics for analysis
        self.pitch_initial_rad = pitch_initial_rad
        self.TSR_initial = TSR_initial
        self.CP_table = CP
        self.CT_table = CT 
        self.CQ_table = CQ

    def load_from_txt(self,fast,txt_filename):
        '''
        Load rotor performance data from a *.txt file. 

        Need to include notes on necessary file format:
        '''
        print('Loading rotor performace data from text file:', txt_filename)

        with open(txt_filename) as pfile:
            for line in pfile:
                # Read Blade Pitch Angles (degrees)
                if 'Pitch angle' in line:
                    pitch_initial = np.array([float(x) for x in pfile.readline().strip().split()])
                    pitch_initial_rad = pitch_initial * deg2rad             # degrees to rad            ! should this be conditional?

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

                # Read Troque Coefficients
                if 'Torque' in line:
                    pfile.readline()
                    Cq = np.empty((len(TSR_initial),len(pitch_initial)))
                    for tsr_i in range(len(TSR_initial)):
                        Cq[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])

            # Store necessary metrics for analysis
            self.pitch_initial_rad = pitch_initial_rad
            self.TSR_initial = TSR_initial
            self.Cp_table = Cp
            self.Ct_table = Ct 
            self.Cq_table = Cq


class RotorPerformance():
    def __init__(self,performance_table, pitch_initial_rad, TSR_initial):
        '''
        Used to find details from rotor performance tables generated by CC-blade or similer BEM-solvers. 
        '''

        # Store performance data tables
        self.performance_table = performance_table          # Table containing rotor performance data, i.e. Cp, Ct, Cq
        self.pitch_initial_rad = pitch_initial_rad          # Pitch angles corresponding to x-axis of performance_table (rad)
        self.TSR_initial = TSR_initial                      # Tip-Speed-Ratios corresponding to y-axis of performance_table (rad)

        # Calculate Gradients
        self.gradient_TSR, self.gradient_pitch = gradient(performance_table)             # gradient_TSR along y-axis, gradient_pitch along x-axis (rows, columns)

        # Optimal below rated TSR and blade pitch
        self.max = np.amax(performance_table)
        self.max_ind = np.where(performance_table == np.amax(performance_table))
        self.TSR_opt = TSR_initial[self.max_ind[0]]
        self.pitch_opt = pitch_initial_rad[self.max_ind[1]]

    def interp_surface(self,pitch,TSR):
        # Form the interpolant functions which can look up any arbitrary location on rotor performance surface
        interp_fun = interpolate.interp2d(self.pitch_initial_rad, self.TSR_initial, self.performance_table, kind='cubic')
        return interp_fun(pitch,TSR)

    def interp_gradient(self,pitch,TSR):
        # Form the interpolant functions to find gradient at any arbitrary location on rotor performance surface
        dCP_beta_interp = interpolate.interp2d(self.pitch_initial_rad, self.TSR_initial, self.gradient_pitch, kind='cubic')
        dCP_TSR_interp = interpolate.interp2d(self.pitch_initial_rad, self.TSR_initial, self.gradient_TSR, kind='cubic')

        # grad.shape output as (2,) numpy array, equivalent to (pitch-direction,TSR-direction)
        grad = np.array([dCP_beta_interp(pitch,TSR), dCP_TSR_interp(pitch,TSR)])
        return np.ndarray.flatten(grad)
    


    # # NOT CERTAIN OF THESE ALTERNATIVES YET
    # def load_from_sowfa(self, fast_folder):
    #     """
    #     Load the parameter files directly from a SOWFA directory
    #     """

    # def load_from_csv(self, fast_folder):
    #     """
    #     Load from a simple CSV file containing the parameters
    #     """