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
from wisdem.ccblade import CCAirfoil, CCBlade
from wisdem.aeroelasticse.FAST_reader import InputReader_OpenFAST
from scipy import interpolate, gradient
import pickle
import matplotlib.pyplot as plt

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class Turbine():
    """
    Class Turbine defines a turbine in terms of what is needed to 
        design the controller and to run the 'tiny' simulation.

    Primary functions (at a high level):
        - Reads an OpenFAST input deck 
        - Runs cc-blade rotor performance analysis
        - Writes a text file containing Cp, Ct, and Cq tables

    Methods:
    -------
    __str__
    load
    save
    load_from_fast
    load_from_ccblade
    load_from_txt
    write_rotor_performance

    Parameters:
    -----------
    turbine_params : dict
                    Dictionary containing known turbine paramaters that are not directly available from OpenFAST input files
    """

    def __init__(self, turbine_params):
        """
        Load turbine parameters from input dictionary
        """
        print('-----------------------------------------------------------------------------')
        print('Loading wind turbine data for NREL\'s ROSCO tuning and simulation processeses')
        print('-----------------------------------------------------------------------------')

        # ------ Turbine Parameters------
        self.rotor_inertia = turbine_params['rotor_inertia']         
        self.rated_rotor_speed = turbine_params['rated_rotor_speed']     
        self.v_min = turbine_params['v_min']                 
        self.v_rated = turbine_params['v_rated']               
        self.v_max = turbine_params['v_max']
        self.max_pitch_rate = turbine_params['max_pitch_rate'] 
        self.min_pitch_rate = -1 * self.max_pitch_rate
        self.max_torque_rate = turbine_params['max_torque_rate']             
        self.rated_power = turbine_params['rated_power']           
        self.bld_edgewise_freq = turbine_params['bld_edgewise_freq']     

    # Allow print out of class
    def __str__(self): 
        '''
        Print some data about the turbine.
        '''
        print('-------------- Turbine Info --------------')
        print('Turbine Name: {}'.format(self.TurbineName))
        print('Rated Power: {} [W]'.format(self.rated_power))
        print('Total Inertia: {:.1f} [kg m^2]'.format(self.J))
        print('Rotor Radius: {:.1f} [m]'.format(self.rotor_radius))
        print('Rated Rotor Speed: {:.1f} [rad/s]'.format(self.rated_rotor_speed))
        print('Max Cp: {:.2f}'.format(self.Cp.max))
        print('------------------------------------------')
        return ' '

    # Save function
    def save(self,filename):
        '''
        Save turbine to pickle

        Parameters:
        ----------
        filename : str
                   Name of file to save pickle 
        # '''
        tuple_to_save = (self)
        pickle.dump( tuple_to_save, open( filename, "wb" ) )

    # Load function
    @staticmethod
    def load(filename):
        '''
        Load turbine from pickle - outdated, but might be okay!!

        Parameters:
        ----------
        filename : str
                   Name of pickle file
        '''
        turbine = pickle.load(open(filename,'rb'))
        return turbine
    # Load data from fast input deck
    def load_from_fast(self, FAST_InputFile,FAST_directory, FAST_ver='OpenFAST',dev_branch=True,rot_source=None, txt_filename=None):
        """
        Load the parameter files directly from a FAST input deck

        Parameters:
        -----------
            Fast_InputFile: str
                            Primary fast model input file (*.fst)
            FAST_directory: str
                            Directory for primary fast model input file
            FAST_ver: string, optional
                      fast version, usually OpenFAST
            dev_branch: bool, optional
                        dev_branch input to InputReader_OpenFAST, probably True
            rot_source: str, optional
                        desired source for rotor to get Cp, Ct, Cq tables. Default is to run cc-blade. 
                            options: cc-blade - run cc-blade
                                     txt - from *.txt file
            txt_filename: str, optional
                          filename for *.txt, only used if rot_source='txt'
        """

        print('Loading FAST model: %s ' % FAST_InputFile)
        self.TurbineName = FAST_InputFile.strip('.fst')
        fast = InputReader_OpenFAST(FAST_ver=FAST_ver,dev_branch=dev_branch)
        fast.FAST_InputFile = FAST_InputFile
        fast.FAST_directory = FAST_directory
        fast.execute()

        if txt_filename:
            self.rotor_performance_filename = txt_filename
        else:
            self.rotor_performance_filename = 'Cp_Ct_Cq.txt'


        # Grab general turbine parameters
        self.TipRad = fast.fst_vt['ElastoDyn']['TipRad']
        self.Rhub =  fast.fst_vt['ElastoDyn']['HubRad']
        self.hubHt = fast.fst_vt['ElastoDyn']['TowerHt'] 
        self.NumBl = fast.fst_vt['ElastoDyn']['NumBl']
        self.TowerHt = fast.fst_vt['ElastoDyn']['TowerHt']
        self.shearExp = 0.2  #HARD CODED FOR NOW
        self.rho = fast.fst_vt['AeroDyn15']['AirDens']
        self.mu = fast.fst_vt['AeroDyn15']['KinVisc']
        self.Ng = fast.fst_vt['ElastoDyn']['GBRatio']
        self.GenEff = fast.fst_vt['ServoDyn']['GenEff']
        self.DTTorSpr = fast.fst_vt['ElastoDyn']['DTTorSpr']
        self.generator_inertia = fast.fst_vt['ElastoDyn']['GenIner']
        self.tilt = fast.fst_vt['ElastoDyn']['ShftTilt'] 
        try:
            self.precone = fast.fst_vt['ElastoDyn']['PreCone1'] # May need to change to PreCone(1) depending on OpenFAST files
        except:
            self.precone = fast.fst_vt['ElastoDyn']['PreCone(1)']
        self.yaw = 0.0
        self.J = self.rotor_inertia + self.generator_inertia * self.Ng**2
        self.rated_torque = self.rated_power/(self.GenEff/100*self.rated_rotor_speed*self.Ng)
        self.rotor_radius = self.TipRad
        # self.omega_dt = np.sqrt(self.DTTorSpr/self.J)

        # Load Cp, Ct, Cq tables
        if rot_source == 'txt':
            self.load_from_txt(txt_filename)
        elif rot_source == 'cc-blade':
            self.load_from_ccblade(fast)
        else:   # default load from cc-blade
            if txt_filename is None:
                print('No rotor performance data source available, running CC-Blade.')
                self.load_from_ccblade(fast)
            elif os.path.exists(txt_filename):
                self.load_from_txt(txt_filename)
            else:
                print('No rotor performance data source available, running CC-Blade.')
                self.load_from_ccblade(fast)

        # Parse rotor performance data
        self.Cp = RotorPerformance(self.Cp_table,self.pitch_initial_rad,self.TSR_initial)
        self.Ct = RotorPerformance(self.Ct_table,self.pitch_initial_rad,self.TSR_initial)
        self.Cq = RotorPerformance(self.Cq_table,self.pitch_initial_rad,self.TSR_initial)

        # Pull out some floating-related data
        wave_tp = fast.fst_vt['HydroDyn']['WaveTp'] 
        try:
            self.wave_peak_period = 1/wave_tp       # Will work if HydroDyn exists and a peak period is defined...
        except:
            self.wave_peak_period = 0.0             # Set as 0.0 when HydroDyn doesn't exist (fixed bottom)

    # Load rotor performance data from CCBlade 
    def load_from_ccblade(self,fast):
        '''
        Loads rotor performance information by running cc-blade aerodynamic analysis. Designed to work with Aerodyn15 blade input files. 

        Parameters:
        -----------
            fast: dict
                  Dictionary containing fast model details - defined using from InputReader_OpenFAST (distributed as a part of AeroelasticSE)

        '''
        print('Loading rotor performace data from CC-Blade.')
        
        # Create CC-Blade Rotor
        r0 = np.array(fast.fst_vt['AeroDynBlade']['BlSpn']) 
        chord0 = np.array(fast.fst_vt['AeroDynBlade']['BlChord'])
        theta0 = np.array(fast.fst_vt['AeroDynBlade']['BlTwist'])
        # -- Adjust for Aerodyn15
        r = r0 + self.Rhub
        chord_intfun = interpolate.interp1d(r0,chord0, bounds_error=None, fill_value='extrapolate', kind='zero')
        chord = chord_intfun(r)
        theta_intfun = interpolate.interp1d(r0,theta0, bounds_error=None, fill_value='extrapolate', kind='zero')
        theta = theta_intfun(r)
        af_idx = np.array(fast.fst_vt['AeroDynBlade']['BlAFID']).astype(int) - 1 #Reset to 0 index
        AFNames = fast.fst_vt['AeroDyn15']['AFNames']   

        # Use airfoil data from FAST file read, assumes AeroDyn 15, assumes 1 Re num per airfoil
        af_dict = {}
        for i, _ in enumerate(fast.fst_vt['AeroDyn15']['af_data']):
            Re    = [fast.fst_vt['AeroDyn15']['af_data'][i]['Re']]
            Alpha = fast.fst_vt['AeroDyn15']['af_data'][i]['Alpha']
            Cl    = fast.fst_vt['AeroDyn15']['af_data'][i]['Cl']
            Cd    = fast.fst_vt['AeroDyn15']['af_data'][i]['Cd']
            Cm    = fast.fst_vt['AeroDyn15']['af_data'][i]['Cm']
            af_dict[i] = CCAirfoil(Alpha, Re, Cl, Cd, Cm)
        # define airfoils for CCBlade
        af = [0]*len(r)
        for i in range(len(r)):
            af[i] = af_dict[af_idx[i]]

        # Now save the CC-Blade rotor
        nSector = 8  # azimuthal discretizations
        self.cc_rotor = CCBlade(r, chord, theta, af, self.Rhub, self.rotor_radius, self.NumBl, rho=self.rho, mu=self.mu,
                        precone=-self.precone, tilt=-self.tilt, yaw=self.yaw, shearExp=self.shearExp, hubHt=self.hubHt, nSector=nSector)
        print('CCBlade initiated successfully.')
        
        # Generate the look-up tables, mesh the grid and flatten the arrays for cc_rotor aerodynamic analysis
        TSR_initial = np.arange(3, 15,0.25)
        pitch_initial = np.arange(-1,25,0.25)
        pitch_initial_rad = pitch_initial * deg2rad
        ws_array = np.ones_like(TSR_initial) * self.v_rated # evaluate at rated wind speed
        omega_array = (TSR_initial * ws_array / self.rotor_radius) * RadSec2rpm
        ws_mesh, pitch_mesh = np.meshgrid(ws_array, pitch_initial)
        ws_flat = ws_mesh.flatten()
        pitch_flat = pitch_mesh.flatten()
        omega_mesh, _ = np.meshgrid(omega_array, pitch_initial)
        omega_flat = omega_mesh.flatten()
        # tsr_flat = (omega_flat * rpm2RadSec * self.rotor_radius)  / ws_flat


        # Get values from cc-blade
        print('Running CCBlade aerodynamic analysis, this may take a minute...')
        # P, T, Q, M, CP, CT, CQ, CM = self.cc_rotor.evaluate(ws_flat, omega_flat, pitch_flat, coefficients=True)
        _, _, _, _, CP, CT, CQ, _ = self.cc_rotor.evaluate(ws_flat, omega_flat, pitch_flat, coefficients=True)
        print('CCBlade aerodynamic analysis run successfully.')

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
        
    def load_from_txt(self,txt_filename):
        '''
        Load rotor performance data from a *.txt file. 

        Parameters:
        -----------
            txt_filename: str
                          Filename of the text containing the Cp, Ct, and Cq data. This should be in the format printed by the write_rotorperformance function
        '''
        print('Loading rotor performace data from text file:', txt_filename)

        with open(txt_filename) as pfile:
            for line in pfile:
                # Read Blade Pitch Angles (degrees)
                if 'Pitch angle' in line:
                    pitch_initial = np.array([float(x) for x in pfile.readline().strip().split()])
                    pitch_initial_rad = pitch_initial * deg2rad             # degrees to rad            -- should this be conditional?

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
    

class RotorPerformance():
    '''
    Class RotorPerformance used to find details from rotor performance 
        tables generated by CC-blade or similer BEM-solvers. 

    Methods:
    --------
    interp_surface
    interp_gradient
    plot_performance

    Parameters:
    -----------
    performance_table : array_like (-)
                        An [n x m] array containing a table of rotor performance data (Cp, Ct, Cq).
    pitch_initial_rad : array_like (rad)
                        An [m x 1] or [1 x m] array containing blade pitch angles corresponding to performance_table. 
    TSR_initial : array_like (rad)
                    An [n x 1] or [1 x n] array containing tip-speed ratios corresponding to  performance_table 
    '''
    def __init__(self,performance_table, pitch_initial_rad, TSR_initial):

        # Store performance data tables
        self.performance_table = performance_table          # Table containing rotor performance data, i.e. Cp, Ct, Cq
        self.pitch_initial_rad = pitch_initial_rad          # Pitch angles corresponding to x-axis of performance_table (rad)
        self.TSR_initial = TSR_initial                      # Tip-Speed-Ratios corresponding to y-axis of performance_table (rad)

        # Calculate Gradients
        self.gradient_TSR, self.gradient_pitch = gradient(performance_table)             # gradient_TSR along y-axis, gradient_pitch along x-axis (rows, columns)

        # "Optimal" below rated TSR and blade pitch (for Cp) - note this may be limited by resolution of Cp-surface
        self.max = np.amax(performance_table)
        self.max_ind = np.where(performance_table == np.amax(performance_table))
        self.pitch_opt = pitch_initial_rad[self.max_ind[1]]
        # --- Find TSR ---
        # Make finer mesh for Tip speed ratios at "optimal" blade pitch angle, do a simple lookup. 
        #       -- nja: this seems to work a little better than interpolating
        performance_beta_max = np.ndarray.flatten(performance_table[:,self.max_ind[1]]) # performance metric at maximizing pitch angle
        TSR_ind = np.arange(0,len(TSR_initial))
        TSR_fine_ind = np.linspace(TSR_initial[0],TSR_initial[-1],(TSR_initial[-1] - TSR_initial[0])*100)
        f_TSR = interpolate.interp1d(TSR_initial,TSR_initial,bounds_error='False',kind='quadratic')    # interpolate function for Cp(tsr) values
        TSR_fine = f_TSR(TSR_fine_ind)
        f_performance = interpolate.interp1d(TSR_initial,performance_beta_max,bounds_error='False',kind='quadratic')    # interpolate function for Cp(tsr) values
        performance_fine = f_performance(TSR_fine_ind)
        performance_max_ind = np.where(performance_fine == np.max(performance_fine))
        self.TSR_opt = float(TSR_fine[performance_max_ind[0]])

    def interp_surface(self,pitch,TSR):
        '''
        2d interpolation to find point on rotor performance surface
        
        Parameters:
        -----------
        pitch : float (rad)
                Pitch angle to look up
        TSR : float (rad)
              Tip-speed ratio to look up
        '''
        
        # Form the interpolant functions which can look up any arbitrary location on rotor performance surface
        interp_fun = interpolate.interp2d(self.pitch_initial_rad, self.TSR_initial, self.performance_table, kind='linear')
        return interp_fun(pitch,TSR)

    def interp_gradient(self,pitch,TSR):
        '''
        2d interpolation to find gradient at a specified point on rotor performance surface
        
        Parameters:
        -----------
        pitch : float (rad)
                Pitch angle to look up
        TSR : float (rad)
              Tip-speed ratio to look up

        Returns:
        --------
        interp_gradient : array_like
                          [1 x 2] array coresponding to gradient in pitch and TSR directions, respectively
        '''
        # Form the interpolant functions to find gradient at any arbitrary location on rotor performance surface
        dCP_beta_interp = interpolate.interp2d(self.pitch_initial_rad, self.TSR_initial, self.gradient_pitch, kind='linear')
        dCP_TSR_interp = interpolate.interp2d(self.pitch_initial_rad, self.TSR_initial, self.gradient_TSR, kind='linear')

        # grad.shape output as (2,) numpy array, equivalent to (pitch-direction,TSR-direction)
        grad = np.array([dCP_beta_interp(pitch,TSR), dCP_TSR_interp(pitch,TSR)])
        return np.ndarray.flatten(grad)
    
    def plot_performance(self,performance_table, pitch_initial_rad, TSR_initial):
        '''
        Plot rotor performance data surface. 
        
        Parameters:
        -----------
        performance_table : array_like (-)
                            An [n x m] array containing a table of rotor performance data (Cp, Ct, Cq) to plot.
        pitch_initial_rad : array_like (rad)
                            An [m x 1] or [1 x m] array containing blade pitch angles corresponding to performance_table (x-axis). 
        TSR_initial : array_like (rad)
                      An [n x 1] or [1 x n] array containing tip-speed ratios corresponding to  performance_table (y-axis)
        '''

        P = plt.contour(pitch_initial_rad * rad2deg, TSR_initial, performance_table, levels=[0.0, 0.3, 0.40, 0.42, 0.44, 0.45, 0.46, 0.47, 0.48,0.481,0.482,0.483,0.484,0.485,0.486,0.487,0.488,0.489, 0.49, 0.491,0.492,0.493,0.494,0.495,0.496,0.497,0.498,0.499, 0.50 ])
        plt.clabel(P, inline=1, fontsize=12)
        plt.title('Power Coefficient', fontsize=14, fontweight='bold')
        plt.xlabel('Pitch Angle [deg]', fontsize=14, fontweight='bold')
        plt.ylabel('TSR [-]', fontsize=14, fontweight='bold')
        plt.xticks(fontsize=12)
        plt.yticks(fontsize=12)
        plt.grid(color=[0.8,0.8,0.8], linestyle='--')
        plt.subplots_adjust(bottom = 0.15, left = 0.15)