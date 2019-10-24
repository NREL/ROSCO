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
import datetime
from ccblade import CCAirfoil, CCBlade
from AeroelasticSE.FAST_reader import InputReader_OpenFAST
from scipy import interpolate, gradient
import pickle

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class Turbine():
    """
    Class turbine defines a turbine in terms of what is needed to design the controller
    and to run the 'tiny' simulation
    """

    def __init__(self, turbine_params):
        """
        Maybe just initialize the internal variables
        This also lists what will need to be defined
        """

        # ------ Turbine Parameters------
        self.rotor_inertia = turbine_params['rotor_inertia']         
        self.rated_rotor_speed = turbine_params['rated_rotor_speed']     
        self.v_min = turbine_params['v_min']                 
        self.v_rated = turbine_params['v_rated']               
        self.v_max = turbine_params['v_max']
        self.max_pitch_rate = turbine_params['max_pitch_rate'] * deg2rad
        self.max_torque_rate = turbine_params['max_torque_rate']             
        self.rated_power = turbine_params['rated_power']           
        self.bld_edgewise_freq = turbine_params['bld_edgewise_freq']     



        # self.rated_rotor_speed = 12.1*rpm2RadSec               # Rated rotor speed (rad/s)
        # # self.rated_rotor_speed = 8.68*rpm2RadSec               # Rated rotor speed (rad/s)
        # # self.rated_rotor_speed = 7.497382*rpm2RadSec               # Rated rotor speed (rad/s)

        # self.v_min = 4.                  # Cut-in wind speed (m/s) (JUST ASSUME FOR NOW)
        # self.v_rated = 11.4                # Rated wind speed (m/s)
        # self.v_max = 25.                  # Cut-out wind speed (m/s), -- Does not need to be exact (JUST ASSUME FOR NOW)
        # self.rated_power = 5000000
        # self.rated_power = 10000000
        # self.rated_power = 15000000
        
        # Init the cc-blade rotor
        self.cc_rotor = None

    # Allow print out of class
    def __str__(self): 

        print('---------------------')
        print('Turbine Info')
        print('J: %.1f' % self.J)
        print('rho: %.1f' % self.rho)
        print('rotor_radius: %.1f' % self.rotor_radius)
        print('---------------------')
        return ' '

    # Save function
    def save(self,filename):
        tuple_to_save = (self.J,self.rho,self.rotor_radius, self.Ng,self.rated_rotor_speed,self.v_min,self.v_rated,self.v_max,self.cc_rotor,self.Cp_table,self.Ct_table,self.Cq_table,self.Cp,self.Ct,self.Cq   )
        pickle.dump( tuple_to_save, open( filename, "wb" ) )

    # Load function
    def load(self, filename):
        self.J,self.rho,self.rotor_radius, self.Ng,self.rated_rotor_speed,self.v_min,self.v_rated,self.v_max,self.cc_rotor,self.cp_interp,self.ct_interp,self.cq_interp = pickle.load(open(filename,'rb'))


    def load_from_fast(self, FAST_InputFile,FAST_directory, FAST_ver='OpenFAST',dev_branch=True,rot_source=None, txt_filename=None):
        """
        Load the parameter files directly from a FAST input deck

        Parameters:
        -----------
            Fast_InputFile: str
                            Primary fast model input file (*.fst)
            FAST_directory: str
                            Directory for primary fast model input file
            drivetrain_intertia: int
                                 drivetrain intertia (kg-m^2)                # nja - this might be able to be automated, see aerodyn.out
            FAST_ver: string, optional
                      fast version, usually OpenFAST
            dev_branch: bool
                        dev_branch input to InputReader_OpenFAST, probably True
            rot_source: str
                        desired source for rotor to get Cp, Ct, Cq tables. Default is to run cc-blade. 
                            options: cc-blade - run cc-blade
                                     txt - from *.txt file
            txt_filename: str
                          filename for *.txt, only used if rot_source='txt'
        """

        print('Loading FAST model: %s ' % FAST_InputFile)
        self.TurbineName = FAST_InputFile.strip('.fst')
        fast = InputReader_OpenFAST(FAST_ver=FAST_ver,dev_branch=dev_branch)
        fast.FAST_InputFile = FAST_InputFile
        fast.FAST_directory = FAST_directory
        fast.execute()


        # Grab general turbine parameters
        self.TipRad = fast.fst_vt['ElastoDyn']['TipRad']
        self.Rhub =  fast.fst_vt['ElastoDyn']['HubRad']
        self.hubHt = fast.fst_vt['ElastoDyn']['TowerHt'] 
        self.NumBl = fast.fst_vt['ElastoDyn']['NumBl']
        self.shearExp = 0.2  #HARD CODED FOR NOW
        self.rho = fast.fst_vt['AeroDyn15']['AirDens']
        self.mu = fast.fst_vt['AeroDyn15']['KinVisc']
        self.Ng = fast.fst_vt['ElastoDyn']['GBRatio']
        self.GenEff = fast.fst_vt['ServoDyn']['GenEff']
        self.DTTorSpr = fast.fst_vt['ElastoDyn']['DTTorSpr']
        self.generator_inertia = fast.fst_vt['ElastoDyn']['GenIner']
        self.tilt = fast.fst_vt['ElastoDyn']['ShftTilt'] 
        self.precone = fast.fst_vt['ElastoDyn']['PreCone1']
        self.yaw = 0.0
        self.J = self.rotor_inertia + self.generator_inertia * self.Ng**2
        self.rated_torque = self.rated_power/(self.GenEff/100*self.rated_rotor_speed*self.Ng)

        # Some additional parameters to save
        self.rotor_radius = self.TipRad
        self.omega_dt = np.sqrt(self.DTTorSpr/self.J)
        
        # if self.rated_rotor_speed:
        #     pass
        # else:
        #     # Calculate rated rotor speed by scaling from NREL 5MW
        #     self.rated_rotor_speed = (63. / self.rotor_radius) * 12.1 * rpm2RadSec

        # Load Cp, Ct, Cq tables
        if rot_source == 'txt':
            self.load_from_txt(fast,txt_filename)
        elif rot_source == 'cc-blade':
            self.load_from_ccblade(fast)
        else:   # default load from cc-blade
            print('No desired rotor performance data source specified, running cc-blade.')
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

        # Used airfoil data from FAST file read, assumes AeroDyn 15, assumes 1 Re num per airfoil
        af_dict = {}
        for i, af_fast in enumerate(fast.fst_vt['AeroDyn15']['af_data']):
        # for i in enumerate(fast.fst_vt['AeroDyn15']['af_data']):
            Re    = [fast.fst_vt['AeroDyn15']['af_data'][i]['Re']]
            Alpha = fast.fst_vt['AeroDyn15']['af_data'][i]['Alpha']
            Cl    = fast.fst_vt['AeroDyn15']['af_data'][i]['Cl']
            Cd    = fast.fst_vt['AeroDyn15']['af_data'][i]['Cd']
            Cm    = fast.fst_vt['AeroDyn15']['af_data'][i]['Cm']
            af_dict[i] = CCAirfoil(Alpha, Re, Cl, Cd, Cm)

        af = [0]*len(r)
        for i in range(len(r)):
            af[i] = af_dict[af_idx[i]]

        # Now save the CC-Blade rotor
        nSector = 8  # azimuthal discretizations
        self.cc_rotor = CCBlade(r, chord, theta, af, self.Rhub, self.rotor_radius, self.NumBl, rho=self.rho, mu=self.mu,
                        precone=self.precone, tilt=self.tilt, yaw=self.yaw, shearExp=self.shearExp, hubHt=self.hubHt, nSector=nSector)


        print('CCBlade run successfully')
        # Generate the look-up tables
        # Mesh the grid and flatten the arrays

        # fixed_rpm = self.rated_rotor_speed*RadSec2rpm # RPM

        TSR_initial = np.arange(3, 15,0.25)
        pitch_initial = np.arange(-1,25,0.25)
        pitch_initial_rad = pitch_initial * deg2rad
        # ws_array = (self.rated_rotor_speed * self.rotor_radius)  / TSR_initial
        ws_array = np.ones_like(TSR_initial) * (self.v_rated - 1.) # 1 m/s below rated wind speed
        omega_array = (TSR_initial * ws_array / self.rotor_radius) * RadSec2rpm
        tsr_array = (omega_array * rpm2RadSec * self.rotor_radius)  / ws_array
        ws_mesh, pitch_mesh = np.meshgrid(ws_array, pitch_initial)
        ws_flat = ws_mesh.flatten()
        pitch_flat = pitch_mesh.flatten()
        omega_mesh, _ = np.meshgrid(omega_array, pitch_initial)
        omega_flat = omega_mesh.flatten()
        # omega_flat = np.ones_like(pitch_flat) * self.rated_rotor_speed * RadSec2rpm
        tsr_flat = (omega_flat * rpm2RadSec * self.rotor_radius)  / ws_flat


        # Get values from cc-blade
        print('Running cc_rotor aerodynamic analysis, this may take a minute...')
        P, T, Q, M, CP, CT, CQ, CM = self.cc_rotor.evaluate(ws_flat, omega_flat, pitch_flat, coefficients=True)

        # Reshape Cp, Ct and Cq
        Cp = np.transpose(np.reshape(CP, (len(pitch_initial), len(TSR_initial))))
        Ct = np.transpose(np.reshape(CT, (len(pitch_initial), len(TSR_initial))))
        Cq = np.transpose(np.reshape(CQ, (len(pitch_initial), len(TSR_initial))))

        # Store necessary metrics for analysis
        self.pitch_initial_rad = pitch_initial_rad
        self.TSR_initial = TSR_initial
        self.Cp_table = Cp
        self.Ct_table = Ct 
        self.Cq_table = Cq
    
    
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

                # Read Torque Coefficients
                if 'Torque' in line:
                    pfile.readline()
                    Cq = np.empty((len(TSR_initial),len(pitch_initial)))
                    for tsr_i in range(len(TSR_initial)):
                        Cq[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])

            # Store necessary metrics for analysis and tuning
            self.pitch_initial_rad = pitch_initial_rad
            self.TSR_initial = TSR_initial
            self.Cp_table = Cp
            self.Ct_table = Ct 
            self.Cq_table = Cq
    
    
    def write_rotorperformance(self,txt_filename='Cp_Ct_Cq.txt'):
        '''
        Write text file containing rotor performance data

        Parameters:
        ------------
        Fill this out...
        '''
        
        file = open(txt_filename,'w')
        file.write('# ----- Rotor performance tables for {}, written on {}'.format(self.TurbineName, now.strftime('%m/%d/%y')))
        file.write('# Pitch angle vector - x axis (matrix columns) (deg)\n')
        for i in range(len(self.Cp.pitch_initial_rad)):
            file.write('%.2f   ' % (self.Cp.pitch_initial_rad[i] * rad2deg))
        file.write('\n# TSR vector - y axis (matrix rows) (-)\n')
        for i in range(len(self.TSR_initial)):
            file.write('%.2f   ' % self.Cp.TSR_initial[i])
        file.write('\n# Wind speed vector - z axis (m/s)\n')
        # for i in range(n_U):
        # --- write arbitrary wind speed for now...
        file.write('%.2f   ' % 3.0)
        file.write('\n')
        
        file.write('\n# Power coefficient\n\n')
        for i in range(len(self.Cp.TSR_initial)):
            for j in range(len(self.Cp.pitch_initial_rad)):
                file.write('%.5f   ' % self.Cp_table[i,j])
            file.write('\n')
        file.write('\n')
        
        file.write('\n#  Thrust coefficient\n\n')
        for i in range(len(self.Ct.TSR_initial)):
            for j in range(len(self.Ct.pitch_initial_rad)):
                file.write('%.5f   ' % self.Ct_table[i,j])
            file.write('\n')
        file.write('\n')
        
        file.write('\n# Torque coefficient\n\n')
        for i in range(len(self.Cq.TSR_initial)):
            for j in range(len(self.Cq.pitch_initial_rad)):
                file.write('%.5f   ' % self.Cq_table[i,j])
            file.write('\n')
        file.write('\n')
            
        file.close()

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
        print('maxind = {}'.format(self.max_ind))
        self.TSR_opt = np.float64(TSR_initial[self.max_ind[0]])
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