# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import datetime
import os
import numpy as np

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class UseOpenFAST():
    ''' 
    A collection of utilities that may be useful for using the tools made accessbile in this toolbox with OpenFAST

    A number of the file processing tools used here were provided by or modified from Emanual Branlard's weio library: https://github.com/ebranlard/weio. 

    Methods:
    --------
    run_openfast
    plot_fast_out
    load_output
    load_ascii_output
    '''
    def __init__(self):
        pass

    def run_openfast(self,fast_dir,fastcall='OpenFAST',fastfile=None,):
        '''
        Runs a openfast openfast simulation 
        Parameters:
        ------------
            fast_dir: string
                    Name of OpenFAST directory containing input files.
            fastcall: string, optional
                    Line used to call openfast when executing from the terminal.
            fastfile: string, optional
                    Filename for *.fst input file. Function will find *.fst if not provided.
        '''

        # Define OpenFAST input filename
        if fastfile:
            print('Using {} to run OpenFAST simulation'.format(fastfile))
        else:
            for file in os.listdir(fast_dir):
                if file.endswith('.fst'):
                    fastfile = file
                    print(file)

        # save starting file path -- note: This is an artifact of needing to call OpenFAST from the same directory as DISCON.IN
        original_path = os.getcwd()
        # change path, run sim
        os.chdir(fast_dir)
        os.system('{} {}'.format(fastcall, os.path.join(fast_dir,'*.fst')))
        # return to original path
        os.chdir(original_path)

    def plot_fast_out(self):
        '''
        Plot OpenFAST outputs 
            - NJA: this is a good place to emulate Post_LoadFastOut.m
        '''

        
    def load_output(self, filenames):
        """Load a FAST binary or ascii output file
        Parameters
        ----------
        filenames : str or list
            filename or list of filenames
        Returns
        -------
        data : ndarray
            data values
        info : dict
            info containing:
                - name: filename
                - description: description of dataset
                - channels: list of channel names
                - attribute_units: list of attribute units
        """
        if type(filenames) is str:
            filenames = [filenames]
            
        data = []
        info = []
        for i, filename in enumerate(filenames):
            assert os.path.isfile(filename), "File, %s, does not exists" % filename
            with open(filename, 'r') as f:
                try:
                    f.readline()
                except UnicodeDecodeError:
                    pass
                    data_bin, info_bin = self.load_binary_output(filename)
                    data.append(data_bin)
                    info.append(info_bin)
                else:
                    data_ascii, info_ascii = self.load_ascii_output(filename)
                    data.append(data_ascii)
                    info.append(info_ascii)
        return info, data

    def load_ascii_output(self, filename):
        '''
        Load FAST ascii output file 
        
        Parameters
        ----------
        filename : str
            filename
        '''
        with open(filename) as f:
            info = {}
            info['name'] = os.path.splitext(os.path.basename(filename))[0]
            # Header is whatever is before the keyword `time`
            in_header = True
            header = []
            while in_header:
                l = f.readline()
                if not l:
                    raise Exception('Error finding the end of FAST out file header. Keyword Time missing.')
                in_header= (l+' dummy').lower().split()[0] != 'time'
                if in_header:
                    header.append(l)
                else:
                    info['description'] = header
                    info['channels'] = l.split()
                    info['attribute_units'] = [unit[1:-1] for unit in f.readline().split()]

            # Data, up to end of file or empty line (potential comment line at the end)
            data = np.array([l.strip().split() for l in takewhile(lambda x: len(x.strip())>0, f.readlines())]).astype(np.float)
            return data, info


    def load_binary_output(self, filename, use_buffer=True):
        """
        
        Info about ReadFASTbinary.m:
        
        Original Author: Bonnie Jonkman, National Renewable Energy Laboratory
        (c) 2012, National Renewable Energy Laboratory
        Edited for FAST v7.02.00b-bjj  22-Oct-2012

        03/09/15: Ported from ReadFASTbinary.m by Mads M Pedersen, DTU Wind
        10/24/18: Low memory/buffered version by E. Branlard, NREL
        11/4/19: Implemented in ROSCO toolbox by N. Abbas, NREL

        Paramters
        ----------
        filename : str
            filename

        Returns
        -------
        data : ndarray
            data values
        info : dict
            info containing:
                - name: filename
                - description: description of dataset
                - channels: list of attribute names
                - attribute_units: list of attribute units
        """
        def fread(fid, n, type):
            fmt, nbytes = {'uint8': ('B', 1), 'int16':('h', 2), 'int32':('i', 4), 'float32':('f', 4), 'float64':('d', 8)}[type]
            return struct.unpack(fmt * n, fid.read(nbytes * n))

        def freadRowOrderTableBuffered(fid, n, type_in, nCols, nOff=0, type_out='float64'):
            """ 
            Reads of row-ordered table from a binary file.
            Read `n` data of type `type_in`, assumed to be a row ordered table of `nCols` columns.
            Memory usage is optimized by allocating the data only once.
            Buffered reading is done for improved performances (in particular for 32bit python)
            `nOff` allows for additional column space at the begining of the storage table.
            Typically, `nOff=1`, provides a column at the beginning to store the time vector.
            @author E.Branlard, NREL
            """
            fmt, nbytes = {'uint8': ('B', 1), 'int16':('h', 2), 'int32':('i', 4), 'float32':('f', 4), 'float64':('d', 8)}[type_in]
            nLines          = int(n/nCols)
            GoodBufferSize  = 4096*40
            nLinesPerBuffer = int(GoodBufferSize/nCols)
            BufferSize      = nCols * nLinesPerBuffer
            nBuffer         = int(n/BufferSize)
            # Allocation of data
            data = np.zeros((nLines,nCols+nOff), dtype = type_out)
            # Reading
            try:
                nIntRead   = 0
                nLinesRead = 0
                while nIntRead<n:
                    nIntToRead = min(n-nIntRead, BufferSize)
                    nLinesToRead = int(nIntToRead/nCols)
                    Buffer = np.array(struct.unpack(fmt * nIntToRead, fid.read(nbytes * nIntToRead)))
                    Buffer = Buffer.reshape(-1,nCols)
                    data[ nLinesRead:(nLinesRead+nLinesToRead),  nOff:(nOff+nCols)  ] = Buffer
                    nLinesRead = nLinesRead + nLinesToRead
                    nIntRead   = nIntRead   + nIntToRead
            except:
                raise Exception('Read only %d of %d values in file:' % (nIntRead, n, filename))
            return data


        FileFmtID_WithTime = 1  #% File identifiers used in FAST
        FileFmtID_WithoutTime = 2
        LenName = 10  #;  % number of characters per channel name
        LenUnit = 10  #;  % number of characters per unit name

        with open(filename, 'rb') as fid:
            FileID = fread(fid, 1, 'int16')[0]  #;             % FAST output file format, INT(2)
            if FileID not in [FileFmtID_WithTime, FileFmtID_WithoutTime]:
                raise Exception('FileID not supported {}. Is it a FAST binary file?'.format(FileID))

            NumOutChans = fread(fid, 1, 'int32')[0]  #;             % The number of output channels, INT(4)
            NT = fread(fid, 1, 'int32')[0]  #;             % The number of time steps, INT(4)


            if FileID == FileFmtID_WithTime:
                TimeScl = fread(fid, 1, 'float64')  #;           % The time slopes for scaling, REAL(8)
                TimeOff = fread(fid, 1, 'float64')  #;           % The time offsets for scaling, REAL(8)
            else:
                TimeOut1 = fread(fid, 1, 'float64')  #;           % The first time in the time series, REAL(8)
                TimeIncr = fread(fid, 1, 'float64')  #;           % The time increment, REAL(8)




            ColScl = fread(fid, NumOutChans, 'float32')  #; % The channel slopes for scaling, REAL(4)
            ColOff = fread(fid, NumOutChans, 'float32')  #; % The channel offsets for scaling, REAL(4)

            LenDesc = fread(fid, 1, 'int32')[0]  #;  % The number of characters in the description string, INT(4)
            DescStrASCII = fread(fid, LenDesc, 'uint8')  #;  % DescStr converted to ASCII
            DescStr = "".join(map(chr, DescStrASCII)).strip()



            ChanName = []  # initialize the ChanName cell array
            for iChan in range(NumOutChans + 1):
                ChanNameASCII = fread(fid, LenName, 'uint8')  #; % ChanName converted to numeric ASCII
                ChanName.append("".join(map(chr, ChanNameASCII)).strip())


            ChanUnit = []  # initialize the ChanUnit cell array
            for iChan in range(NumOutChans + 1):
                ChanUnitASCII = fread(fid, LenUnit, 'uint8')  #; % ChanUnit converted to numeric ASCII
                ChanUnit.append("".join(map(chr, ChanUnitASCII)).strip()[1:-1])


            #    %-------------------------
            #    % get the channel time series
            #    %-------------------------

            nPts = NT * NumOutChans  #;           % number of data points in the file
            #print('NT',NT)
            #print('NumOutChans',NumOutChans)


            if FileID == FileFmtID_WithTime:
                PackedTime = fread(fid, NT, 'int32')  #; % read the time data
                cnt = len(PackedTime)
                if cnt < NT:
                    raise Exception('Could not read entire %s file: read %d of %d time values' % (filename, cnt, NT))

            if use_buffer:
                # Reading data using buffers, and allowing an offset for time column (nOff=1)
                data = freadRowOrderTableBuffered(fid, nPts, 'int16', NumOutChans, nOff=1, type_out='float64')
            else:
                # NOTE: unpacking huge data not possible on 32bit machines
                PackedData = fread(fid, nPts, 'int16')  #; % read the channel data
                cnt = len(PackedData)
                if cnt < nPts:
                    raise Exception('Could not read entire %s file: read %d of %d values' % (filename, cnt, nPts))
                data = np.array(PackedData).reshape(NT, NumOutChans)
                del PackedData

        if FileID == FileFmtID_WithTime:
            time = (np.array(PackedTime) - TimeOff) / TimeScl;
        else:
            time = TimeOut1 + TimeIncr * np.arange(NT)

        #import pdb
        #pdb.set_trace()

        #    %-------------------------
        #    % Scale the packed binary to real data
        #    %-------------------------
        if use_buffer:
            # Scaling Data
            for iCol in range(NumOutChans):
                if np.isnan(ColScl[iCol]) and np.isnan(ColOff[iCol]):
                    data[:,iCol+1] = 0 # probably due to a division by zero in Fortran
                else:
                    data[:,iCol+1] = (data[:,iCol+1] - ColOff[iCol]) / ColScl[iCol]
            # Adding time column
            data[:,0] = time
        else:
            # NOTE: memory expensive due to time conversion, and concatenation
            data = (data - ColOff) / ColScl
            data = np.concatenate([time.reshape(NT, 1), data], 1)

        info = {'name': os.path.splitext(os.path.basename(filename))[0],
                'description': DescStr,
                'channels': ChanName,
                'attribute_units': ChanUnit}
        return data, info


class FileProcessing():
    """
    Class FileProcessing used to write out controller 
        parameter files need to run ROSCO

    Methods:
    -----------
    write_param_file
    write_rotor_performance
    """

    def __init__(self):
        pass
    def write_param_file(self, turbine, controller, param_file='DISCON.IN', txt_filename='Cp_Ct_Cq.txt'):
        """
        Print the controller parameters to the DISCON.IN input file for the generic controller

        Parameters:
        -----------
        turbine: class
                 Turbine class containing turbine operation information (ref speeds, etc...)
        controller: class
                    Controller class containing controller operation information (gains, etc...)
        param_file: str, optional
            filename for parameter input file, should be DISCON.IN
        txt_filename: str, optional
                      filename of rotor performance file
        """
        print('Writing new controller parameter file parameter file: %s.' % param_file)
        # Should be obvious what's going on here...
        file = open(param_file,'w')
        file.write('! Controller parameter input file for the %s wind turbine\n' % turbine.TurbineName)
        file.write('!    - File written using NREL Reference OpenSource Controller tuning logic on %s\n' % now.strftime('%m/%d/%y'))
        file.write('\n')
        file.write('!------- DEBUG ------------------------------------------------------------\n')
        file.write('{0:<12d}        ! LoggingLevel		- {{0: write no debug files, 1: write standard output .dbg-file, 2: write standard output .dbg-file and complete avrSWAP-array .dbg2-file}}\n'.format(controller.LoggingLevel))
        file.write('\n')
        file.write('!------- CONTROLLER FLAGS -------------------------------------------------\n')
        file.write('{0:<12d}        ! F_LPFType			- {{1: first-order low-pass filter, 2: second-order low-pass filter}}, [rad/s] (currently filters generator speed and pitch control signals\n'.format(controller.F_LPFType))
        file.write('{0:<12d}        ! F_NotchType		- Notch on the measured generator speed {{0: disable, 1: enable}}\n'.format(controller.F_NotchType))
        file.write('{0:<12d}        ! IPC_ControlMode	- Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {{0: off, 1: 1P reductions, 2: 1P+2P reductions}}\n'.format(controller.IPC_ControlMode))
        file.write('{0:<12d}        ! VS_ControlMode	- Generator torque control mode in above rated conditions {{0: constant torque, 1: constant power, 2: TSR tracking PI control}}\n'.format(controller.VS_ControlMode))
        file.write('{0:<12d}        ! PC_ControlMode    - Blade pitch control mode {{0: No pitch, fix to fine pitch, 1: active PI blade pitch control}}\n'.format(controller.PC_ControlMode))
        file.write('{0:<12d}        ! Y_ControlMode		- Yaw control mode {{0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC}}\n'.format(controller.Y_ControlMode))
        file.write('{0:<12d}        ! SS_Mode           - Setpoint Smoother mode {{0: no setpoint smoothing, 1: introduce setpoint smoothing}}\n'.format(controller.SS_Mode))
        file.write('{0:<12d}        ! WE_Mode           - Wind speed estimator mode {{0: One-second low pass filtered hub height wind speed, 1: Immersion and Invariance Estimator, 2: Extended Kalman Filter}}\n'.format(controller.WE_Mode))
        file.write('{0:<12d}        ! PS_Mode           - Peak shaving mode {{0: no peak shaving, 1: implement peak shaving}}\n'.format(controller.PS_Mode))
        file.write('\n')
        file.write('!------- FILTERS ----------------------------------------------------------\n') 
        file.write('{:<13.2f}       ! F_LPFCornerFreq	- Corner frequency (-3dB point) in the low-pass filters, [rad/s]\n'.format(turbine.bld_edgewise_freq * 1/4)) 
        file.write('{:<13.2f}       ! F_LPFDamping		- Damping coefficient [used only when F_FilterType = 2]\n'.format(controller.F_LPFDamping))
        file.write('{:<13.2f}       ! F_NotchCornerFreq	- Natural frequency of the notch filter, [rad/s]\n'.format(0.0))
        file.write('{:<6.1f}{:<13.1f} ! F_NotchBetaNumDen	- Two notch damping values (numerator and denominator, resp) - determines the width and depth of the notch, [-]\n'.format(0.0,0.0))
        file.write('{:<014.6f}      ! F_SSCornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother, [rad/s].\n'.format(controller.ss_cornerfreq))
        file.write('\n')
        file.write('!------- BLADE PITCH CONTROL ----------------------------------------------\n')
        file.write('{:<11d}         ! PC_GS_n			- Amount of gain-scheduling table entries\n'.format(len(controller.pitch_op_pc)))
        file.write('{}              ! PC_GS_angles	    - Gain-schedule table: pitch angles\n'.format(''.join('{:<4.6f}  '.format(controller.pitch_op_pc[i]) for i in range(len(controller.pitch_op_pc)))))            
        file.write('{}              ! PC_GS_KP		- Gain-schedule table: pitch controller kp gains\n'.format(''.join('{:<4.6f}  '.format(controller.pc_gain_schedule.Kp[i]) for i in range(len(controller.pc_gain_schedule.Kp)))))
        file.write('{}              ! PC_GS_KI		- Gain-schedule table: pitch controller ki gains\n'.format(''.join('{:<4.6f}  '.format(controller.pc_gain_schedule.Ki[i]) for i in range(len(controller.pc_gain_schedule.Ki)))))
        file.write('{}              ! PC_GS_KD			- Gain-schedule table: pitch controller kd gains\n'.format(''.join('{:<1.1f}  '.format(0.0) for i in range(len(controller.pc_gain_schedule.Ki)))))
        file.write('{}              ! PC_GS_TF			- Gain-schedule table: pitch controller tf gains (derivative filter)\n'.format(''.join('{:<1.1f}  '.format(0.0) for i in range(len(controller.pc_gain_schedule.Ki)))))
        file.write('{:<014.5f}      ! PC_MaxPit			- Maximum physical pitch limit, [rad].\n'.format(controller.max_pitch))
        file.write('{:<014.5f}      ! PC_MinPit			- Minimum physical pitch limit, [rad].\n'.format(controller.min_pitch))
        file.write('{:<014.5f}      ! PC_MaxRat			- Maximum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(turbine.max_pitch_rate))
        file.write('{:<014.5f}      ! PC_MinRat			- Minimum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(turbine.min_pitch_rate))
        file.write('{:<014.5f}      ! PC_RefSpd			- Desired (reference) HSS speed for pitch controller, [rad/s].\n'.format(turbine.rated_rotor_speed*turbine.Ng))
        file.write('{:<014.5f}      ! PC_FinePit		- Record 5: Below-rated pitch angle set-point, [rad]\n'.format(controller.min_pitch))
        file.write('{:<014.5f}      ! PC_Switch			- Angle above lowest minimum pitch angle for switch, [rad]\n'.format(1 * deg2rad))
        file.write('{:<11d}         ! Z_EnableSine		- Enable/disable sine pitch excitation, used to validate for dynamic induction control, will be removed later, [-]\n'.format(0))
        file.write('{:<014.5f}      ! Z_PitchAmplitude	- Amplitude of sine pitch excitation, [rad]\n'.format(0.0))
        file.write('{:<014.5f}      ! Z_PitchFrequency	- Frequency of sine pitch excitation, [rad/s]\n'.format(0.0))
        file.write('\n')
        file.write('!------- INDIVIDUAL PITCH CONTROL -----------------------------------------\n')
        file.write('{:<13.1f}       ! IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from IPC), [rad]\n'.format(0.0))
        file.write('{:<6.1f}{:<13.1f} ! IPC_KI			- Integral gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n'.format(0.0,0.0))
        file.write('{:<6.1f}{:<13.1f} ! IPC_aziOffset		- Phase offset added to the azimuth angle for the individual pitch controller, [rad]. \n'.format(0.0,0.0))
        file.write('{:<13.1f}       ! IPC_CornerFreqAct - Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {{0: Disable}}, [rad/s]\n'.format(0.0))
        file.write('\n')
        file.write('!------- VS TORQUE CONTROL ------------------------------------------------\n')
        file.write('{:<014.5f}      ! VS_GenEff			- Generator efficiency mechanical power -> electrical power, [should match the efficiency defined in the generator properties!], [-]\n'.format(turbine.GenEff))
        file.write('{:<014.5f}      ! VS_ArSatTq		- Above rated generator torque PI control saturation, [Nm]\n'.format(turbine.rated_torque))
        file.write('{:<014.5f}      ! VS_MaxRat			- Maximum torque rate (in absolute value) in torque controller, [Nm/s].\n'.format(turbine.max_torque_rate))
        file.write('{:<014.5f}      ! VS_MaxTq			- Maximum generator torque in Region 3 (HSS side), [Nm].\n'.format(turbine.rated_torque*1.1))
        file.write('{:<014.5f}      ! VS_MinTq			- Minimum generator (HSS side), [Nm].\n'.format(0.0))
        file.write('{:<014.5f}      ! VS_MinOMSpd		- Optimal mode minimum speed, cut-in speed towards optimal mode gain path, [rad/s]\n'.format(controller.vs_minspd))
        file.write('{:<014.5f}      ! VS_Rgn2K			- Generator torque constant in Region 2 (HSS side), [N-m/(rad/s)^2]\n'.format(controller.vs_rgn2K))
        file.write('{:<014.5f}      ! VS_RtPwr			- Wind turbine rated power [W]\n'.format(turbine.rated_power))
        file.write('{:<014.5f}      ! VS_RtTq			- Rated torque, [Nm].\n'.format(turbine.rated_torque))
        file.write('{:<014.5f}      ! VS_RefSpd			- Rated generator speed [rad/s]\n'.format(controller.vs_refspd))
        file.write('{:<11d}         ! VS_n				- Number of generator PI torque controller gains\n'.format(1))
        file.write('{:<014.5f}      ! VS_KP				- Proportional gain for generator PI torque controller [1/(rad/s) Nm]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(controller.vs_gain_schedule.Kp[-1]))
        file.write('{:<014.5f}      ! VS_KI				- Integral gain for generator PI torque controller [1/rad Nm]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(controller.vs_gain_schedule.Ki[-1]))
        file.write('{:<13.2f}       ! VS_TSRopt			- Power-maximizing region 2 tip-speed-ratio [rad].\n'.format(turbine.Cp.TSR_opt))
        file.write('\n')
        file.write('!------- SETPOINT SMOOTHER ---------------------------------------------\n')
        file.write('{:<13.5f}       ! SS_VSGain         - Variable speed torque controller setpoint smoother gain, [-].\n'.format(controller.ss_vsgain))
        file.write('{:<13.5f}       ! SS_PCGain         - Collective pitch controller setpoint smoother gain, [-].\n'.format(controller.ss_pcgain))
        file.write('\n')
        file.write('!------- WIND SPEED ESTIMATOR ---------------------------------------------\n')
        file.write('{:<13.3f}       ! WE_BladeRadius	- Blade length [m]\n'.format(turbine.rotor_radius))
        file.write('{:<11d}         ! WE_CP_n			- Amount of parameters in the Cp array\n'.format(1))
        file.write(          '{}    ! WE_CP - Parameters that define the parameterized CP(lambda) function\n'.format(''.join('{:<2.1f} '.format(0.0) for i in range(4))))
        file.write('{:<13.1f}		! WE_Gamma			- Adaption gain of the wind speed estimator algorithm [m/rad]\n'.format(0.0))
        file.write('{:<13.1f}       ! WE_GearboxRatio	- Gearbox ratio [>=1],  [-]\n'.format(turbine.Ng))
        file.write('{:<014.5f}      ! WE_Jtot			- Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, [kg m^2]\n'.format(turbine.J))
        file.write('{:<13.3f}       ! WE_RhoAir			- Air density, [kg m^-3]\n'.format(turbine.rho))
        file.write(      '"{}"      ! PerfFileName      - File containing rotor performance tables (Cp,Ct,Cq)\n'.format(turbine.rotor_performance_filename))
        file.write('{:<7d} {:<10d}  ! PerfTableSize     - Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios\n'.format(len(turbine.Cp.pitch_initial_rad),len(turbine.Cp.TSR_initial)))
        file.write('{:<11d}         ! WE_FOPoles_N      - Number of first-order system poles used in EKF\n'.format(len(controller.A)))
        file.write('{}              ! WE_FOPoles_v      - Wind speeds corresponding to first-order system poles [m/s]\n'.format(''.join('{:<4.2f} '.format(controller.v[i]) for i in range(len(controller.v)))))
        file.write('{}              ! WE_FOPoles        - First order system poles\n'.format(''.join('{:<10.8f} '.format(controller.A[i]) for i in range(len(controller.A)))))
        file.write('\n')
        file.write('!------- YAW CONTROL ------------------------------------------------------\n')
        file.write('{:<13.1f}       ! Y_ErrThresh		- Yaw error threshold. Turbine begins to yaw when it passes this. [rad^2 s]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]\n'.format(0.0))
        file.write('{:<11d}         ! Y_IPC_n			- Number of controller gains (yaw-by-IPC)\n'.format(1))
        file.write('{:<13.1f}       ! Y_IPC_KP			- Yaw-by-IPC proportional controller gain Kp\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_KI			- Yaw-by-IPC integral controller gain Ki\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_omegaLP		- Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_IPC_zetaLP		- Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_MErrSet			- Yaw alignment error, set point [rad]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_omegaLPFast		- Corner frequency fast low pass filter, 1.0 [Hz]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_omegaLPSlow		- Corner frequency slow low pass filter, 1/60 [Hz]\n'.format(0.0))
        file.write('{:<13.1f}       ! Y_Rate			- Yaw rate [rad/s]\n'.format(0.0))
        file.write('\n')
        file.write('!------- TOWER FORE-AFT DAMPING -------------------------------------------\n')
        file.write('{:<11d}         ! FA_KI				- Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m] - !NJA - Make this a flag\n'.format(-1))
        file.write('{:<13.1f}       ! FA_HPF_CornerFreq	- Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]\n'.format(0.0))
        file.write('{:<13.1f}       ! FA_IntSat			- Integrator saturation (maximum signal amplitude contribution to pitch from FA damper), [rad]\n'.format(0.0))
        file.write('\n')
        file.write('!------- PEAK SHAVING -------------------------------------------\n')
        file.write('{:<11d}         ! PS_BldPitchMin_N  - Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)\n'.format(len(controller.ps.pitch_min)))
        file.write('{}              ! PS_WindSpeeds       - Wind speeds corresponding to minimum blade pitch angles [m/s]\n'.format(''.join('{:<4.2f} '.format(controller.ps.v[i]) for i in range(len(controller.ps.v)))))
        file.write('{}              ! PS_BldPitchMin          - Minimum blade pitch angles [rad]'.format(''.join('{:<10.8f} '.format(controller.ps.pitch_min[i]) for i in range(len(controller.ps.pitch_min)))))
        file.close()

    def write_rotor_performance(self,turbine,txt_filename='Cp_Ct_Cq.txt'):
        '''
        Write text file containing rotor performance data

        Parameters:
        ------------
            txt_filename: str, optional
                          Desired output filename to print rotor performance data. Default is Cp_Ct_Cq.txt
        '''
        print('Writing rotor performance text file: {}'.format(txt_filename))
        file = open(txt_filename,'w')
        # Headerlines
        file.write('# ----- Rotor performance tables for the {} wind turbine ----- \n'.format(turbine.TurbineName))
        file.write('# ------------ Written on {} using the ROSCO toolbox ------------ \n\n'.format(now.strftime('%b-%d-%y')))

        # Pitch angles, TSR, and wind speed
        file.write('# Pitch angle vector - x axis (matrix columns) (deg)\n')
        for i in range(len(turbine.Cp.pitch_initial_rad)):
            file.write('{:0.4}   '.format(turbine.Cp.pitch_initial_rad[i] * rad2deg))
        file.write('\n# TSR vector - y axis (matrix rows) (-)\n')
        for i in range(len(turbine.TSR_initial)):
            file.write('{:0.4}    '.format(turbine.Cp.TSR_initial[i]))
        file.write('\n# Wind speed vector - z axis (m/s)\n')
        file.write('{:0.4}    '.format(turbine.v_rated))
        file.write('\n')
        
        # Cp
        file.write('\n# Power coefficient\n\n')
        for i in range(len(turbine.Cp.TSR_initial)):
            for j in range(len(turbine.Cp.pitch_initial_rad)):
                file.write('{0:.6f}   '.format(turbine.Cp_table[i,j]))
            file.write('\n')
        file.write('\n')
        
        # Ct
        file.write('\n#  Thrust coefficient\n\n')
        for i in range(len(turbine.Ct.TSR_initial)):
            for j in range(len(turbine.Ct.pitch_initial_rad)):
                file.write('{0:.6f}   '.format(turbine.Ct_table[i,j]))
            file.write('\n')
        file.write('\n')
        
        # Cq
        file.write('\n# Torque coefficient\n\n')
        for i in range(len(turbine.Cq.TSR_initial)):
            for j in range(len(turbine.Cq.pitch_initial_rad)):
                file.write('{0:.6f}   '.format(turbine.Cq_table[i,j]))
            file.write('\n')
        file.write('\n')
        file.close()