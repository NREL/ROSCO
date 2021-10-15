''' Class for defining linear model 

Will have an A, B, C, and D matrix for every operating point
u_op, y_op, and x_op

'''

import os
import numpy as np
import scipy as sp
import control as co
import matplotlib.pyplot as plt
import multiprocessing as mp
from itertools import chain
from scipy.io import loadmat

try:
    import pyFAST.linearization.mbc.mbc3 as mbc
except ImportError:
    import weis.control.mbc.mbc3 as mbc
except ImportError:
    raise ImportError('Unable to load mbc3 from pyFAST or WEIS')

class LinearTurbineModel(object):

    def __init__(self, lin_file_dir, lin_file_names, nlin=12, reduceStates=False, fromMat=False, lin_file=None, rm_hydro=False, load_parallel=False):
        '''
            inputs:    
                lin_file_dir (string) - directory of linear file outputs from OpenFAST
                lin_file_names (list of strings) - name of output file from OpenFAST, without extension
                nlin (integer) - number of linearization points
                reduceStates (bool) - State space reduction currently unsupported N/A
                fromMat (bool) - load from .mat file
                lin_file (string) -  name of .mat file from matlab version of mbc3, only used if fromMat=True
                rm_hydro (bool) - remove hydrodynamic states
                load_parallel (bool) - run mbc3 usying multiprocessing
        '''
        if not fromMat:

            # Number of linearization cases or OpenFAST sims, different from nlin/NLinTimes in OpenFAST
            n_lin_cases = len(lin_file_names)

            u_ops = np.array([], [])
            all_MBC = []
            all_matData = []
            all_FAST_linData = []

            if load_parallel:
                import time
                t1 = time.time()
                all_linfiles = [[os.path.realpath(os.path.join(
                    lin_file_dir, lin_file_names[iCase] + '.{}.lin'.format(i_lin+1))) for i_lin in range(0, nlin)] for iCase in range(0,n_lin_cases)]
                cores = mp.cpu_count()
                pool = mp.Pool(cores)
                all_MBC, all_matData, all_FAST_linData = zip(*pool.map(run_mbc3, all_linfiles))
                pool.close()
                pool.join()
                print('loaded in parallel in {} seconds'.format(time.time()-t1))
            else:
                import time
                t1 = time.time()
                for iCase in range(0, n_lin_cases):
                    lin_files_i = [os.path.realpath(os.path.join(
                        lin_file_dir, lin_file_names[iCase] + '.{}.lin'.format(i_lin+1))) for i_lin in range(0, nlin)]
                    MBC, matData, FAST_linData = run_mbc3(lin_files_i)
                    all_MBC.append(MBC)
                    all_matData.append(matData)
                    all_FAST_linData.append(FAST_linData)
                print('loaded in serial in {} seconds'.format(time.time()-t1))


            for iCase in range(0, n_lin_cases):
                # nlin array of linearization outputs for iCase

                MBC = all_MBC[iCase]
                matData = all_matData[iCase]
                FAST_linData = all_FAST_linData[iCase]

                if not iCase:   # first time through
                    # Initialize operating points, matrices
                    u_ops = np.zeros((matData['NumInputs'], n_lin_cases))
                    y_ops = np.zeros((matData['NumOutputs'], n_lin_cases))
                    x_ops = np.zeros((matData['NumStates'], n_lin_cases))
                    omega = np.zeros((nlin, n_lin_cases))

                # operating points
                # u_ops \in real(n_inps,n_ops)
                u_ops[:, iCase] = np.mean(matData['u_op'], 1)

                # y_ops \in real(n_outs,n_ops)
                y_ops[:, iCase] = np.mean(matData['y_op'], 1)

                # x_ops \in real(n_states,n_ops), note this is un-reduced state space model states, with all hydro states
                x_ops[:, iCase] = np.mean(matData['xop'], 1)

                # store rotor speed in rpm of each linearization, omega \in real(n_linear,n_cases)
                omega[:, iCase] = matData['Omega'] * 60 / (2 * np.pi)

                # keep all inputs and outputs
                indInps = np.arange(0, matData['NumInputs']).reshape(-1, 1)
                indOuts = np.arange(0, matData['NumOutputs']).reshape(-1, 1)
                indStates = np.arange(0, matData['NumStates'])

                if not iCase:
                    A_ops = np.zeros((len(indStates), len(indStates), n_lin_cases))
                    B_ops = np.zeros((len(indStates), len(indInps), n_lin_cases))
                    C_ops = np.zeros((len(indOuts), len(indStates), n_lin_cases))
                    D_ops = np.zeros((len(indOuts), len(indInps), n_lin_cases))
                    WindSpeeds = np.zeros(n_lin_cases)

                # A \in real(n_states,n_states,n_ops)
                A_ops[:, :, iCase] = MBC['AvgA'] 
                # B \in real(n_states,n_inputs,n_ops)
                B_ops[:, :, iCase] = MBC['AvgB'] 
                # C \in real(n_outs,n_states,n_ops)
                C_ops[:, :, iCase] = MBC['AvgC'] 
                # D \in real(n_outs,n_inputs,n_ops)
                D_ops[:, :, iCase] = MBC['AvgD']

                # if reduceStates:
                #     if not iCase:
                #         n_states = np.zeros((n_lin_cases, 1), dtype=int)
                #     P = co.StateSpace(A_ops[:, :, iCase], B_ops[:, :, iCase],
                #                       C_ops[:, :, iCase], D_ops[:, :, iCase], remove_useless=False)

                #     # figure out minimum number of states for the systems at each operating point
                #     P_min = co.minreal(P, tol=1e-7, verbose=False)
                #     n_states[iCase] = P_min.states

            # # Now loop through and reduce number of states to maximum of n_states
            # # This is broken!!  Works fine if reduceStates = False and isn't problematic to have all the extra states...yet
            if reduceStates:
                raise RuntimeError('reduceStates=True is not currently supported.')
            #     for iCase in range(0, n_lin_cases):
            #         if not iCase:
            #             self.A_ops = np.zeros((max(n_states)[0], max(n_states)[0], n_lin_cases))
            #             self.B_ops = np.zeros((max(n_states)[0], len(indInps), n_lin_cases))
            #             self.C_ops = np.zeros((len(indOuts), max(n_states)[0], n_lin_cases))
            #             self.D_ops = np.zeros((len(indOuts), len(indInps), n_lin_cases))

            #         P = co.StateSpace(A_ops[:, :, iCase], B_ops[:, :, iCase],
            #                           C_ops[:, :, iCase], D_ops[:, :, iCase], remove_useless=False)
            #         # I don't know why it's not reducing to max(n_states), must add 2
            #         P_red = co.balred(P, max(n_states)[0], method='matchdc')
            #         # P_red = co.minreal(P,tol=1e-7,verbose=False)         # I don't know why it's not reducing to max(n_states), must add 2
            #         self.A_ops[:, :, iCase] = P_red.A
            #         self.B_ops[:, :, iCase] = P_red.B
            #         self.C_ops[:, :, iCase] = P_red.C
            #         self.D_ops[:, :, iCase] = P_red.D

            else:
                self.A_ops = A_ops
                self.B_ops = B_ops
                self.C_ops = C_ops
                self.D_ops = D_ops
            
            # Convert output RPM to rad/s
            rpm_idx = np.flatnonzero(np.core.defchararray.find(matData['DescOutput'], 'rpm') > -1).tolist()
            self.C_ops[rpm_idx, :, :] = rpm2radps(self.C_ops[rpm_idx, :, :])
            matData['DescOutput'] = [desc.replace('rpm', 'rad/s') for desc in matData['DescOutput']]
            # Convert output deg to rad
            deg_idx = np.flatnonzero(np.core.defchararray.find(matData['DescOutput'], 'deg') > -1).tolist()
            self.C_ops[deg_idx, :, :] = deg2rad(self.C_ops[deg_idx, :, :])
            matData['DescOutput'] = [desc.replace('deg', 'rad') for desc in matData['DescOutput']]


            # Save wind speed as own array since that's what we'll schedule over to start
            self.u_ops = u_ops
            self.u_h = self.u_ops[0]
            self.y_ops = y_ops
            self.x_ops = x_ops

            # Input/Output Indices
            self.ind_fast_inps = indInps.squeeze()
            self.ind_fast_outs = indOuts.squeeze()

            # Input, Output, State Descriptions
            self.DescCntrlInpt  = matData['DescCntrlInpt']
            self.DescStates     = matData['DescStates']
            self.DescOutput     = matData['DescOutput']
            self.StateDerivOrder = matData['StateDerivOrder']
            self.omega_rpm      = omega

            # Other important things
            self.n_lin_cases = n_lin_cases

            # Trim the system
            self.trim_system(rm_azimuth=True, rm_hydro=rm_hydro)


        else:  # from matlab .mat file m
            matDict = loadmat(lin_file)

            # operating points
            # u_ops \in real(n_inps,n_ops)
            u_ops = np.zeros(matDict['u_ops'][0][0].shape)
            for u_op in matDict['u_ops'][0]:
                u_ops = np.concatenate((u_ops, u_op), axis=1)

            self.u_ops = u_ops[:, 1:]

            # y_ops \in real(n_outs,n_ops)
            y_ops = np.zeros(matDict['y_ops'][0][0].shape)
            for y_op in matDict['y_ops'][0]:
                y_ops = np.concatenate((y_ops, y_op), axis=1)

            self.y_ops = y_ops[:, 1:]

            # x_ops \in real(n_states,n_ops), note this is un-reduced state space model states, with all hydro states
            x_ops = np.zeros(matDict['x_ops'][0][0].shape)
            for x_op in matDict['x_ops'][0]:
                x_ops = np.concatenate((x_ops, x_op), axis=1)

            self.x_ops = x_ops[:, 1:]

            # Matrices

            # A \in real(n_states,n_states,n_ops)
            n_states = np.shape(matDict['A'][0][0])[0]
            A_ops = np.zeros((n_states, n_states, 1))
            for A_op in matDict['A'][0]:
                A_ops = np.concatenate((A_ops, np.expand_dims(A_op, 2)), axis=2)

            self.A_ops = A_ops[:, :, 1:]

            # B \in real(n_states,n_inputs,n_ops)
            n_states = np.shape(matDict['B'][0][0])[0]
            n_inputs = np.shape(matDict['B'][0][0])[1]
            B_ops = np.zeros((n_states, n_inputs, 1))
            for B_op in matDict['B'][0]:
                B_ops = np.concatenate((B_ops, np.expand_dims(B_op, 2)), axis=2)

            self.B_ops = B_ops[:, :, 1:]

            # C \in real(n_outs,n_states,n_ops)
            n_states = np.shape(matDict['C'][0][0])[1]
            n_outs = np.shape(matDict['C'][0][0])[0]
            C_ops = np.zeros((n_outs, n_states, 1))
            for C_op in matDict['C'][0]:
                C_ops = np.concatenate((C_ops, np.expand_dims(C_op, 2)), axis=2)

            self.C_ops = C_ops[:, :, 1:]

            # D \in real(n_outs,n_inputs,n_ops)
            n_states = np.shape(matDict['D'][0][0])[1]
            n_outs = np.shape(matDict['D'][0][0])[0]
            D_ops = np.zeros((n_outs, n_inputs, 1))
            for D_op in matDict['D'][0]:
                D_ops = np.concatenate((D_ops, np.expand_dims(D_op, 2)), axis=2)

            self.D_ops = D_ops[:, :, 1:]

            # Save wind speed as own array since that's what we'll schedule over to start
            self.u_h = self.u_ops[0]

            # Input/Output Indices
            self.ind_fast_inps = matDict['indInps'][0] - 1

            self.ind_fast_outs = matDict['indOuts'][0] - 1
            # self.ind_fast_outs       = matDict['indOuts'][0][0] - 1


    def get_plant_op(self, u_rot, reduce_states):
        '''
        Interpolate system matrices using wind speed operating point wind_speed_op = mean(u_rot)

        inputs: u_rot timeseries of wind speeds

        '''

        wind_speed_op = np.mean(u_rot)

        if len(self.u_h) > 1:
            f_A = sp.interpolate.interp1d(self.u_h, self.A_ops)
            f_B = sp.interpolate.interp1d(self.u_h, self.B_ops)
            f_C = sp.interpolate.interp1d(self.u_h, self.C_ops)
            f_D = sp.interpolate.interp1d(self.u_h, self.D_ops)

            f_u = sp.interpolate.interp1d(self.u_h, self.u_ops)
            f_y = sp.interpolate.interp1d(self.u_h, self.y_ops)
            f_x = sp.interpolate.interp1d(self.u_h, self.x_ops)

            if wind_speed_op < self.u_h.min() or wind_speed_op > self.u_h.max():
                print('WARNING: Requested linearization operating point outside of range')
                print('Simulation operating point is ' + str(wind_speed_op) + ' m/s')
                print('Linearization operating points from ' +
                      str(self.u_h.min()) + ' to ' + str(self.u_h.max()) + ' m/s')
                print('Results may not be as expected')

            if wind_speed_op < self.u_h.min():
                wind_speed_op = self.u_h.min()

            elif wind_speed_op > self.u_h.max():
                wind_speed_op = self.u_h.max()

            A = f_A(wind_speed_op)
            B = f_B(wind_speed_op)
            C = f_C(wind_speed_op)
            D = f_D(wind_speed_op)

            u_op = f_u(wind_speed_op)
            x_op = f_x(wind_speed_op)
            y_op = f_y(wind_speed_op)
        else:
            print('WARNING: Only one linearization, at ' + str(self.u_h[0]) + ' m/s')
            print('Simulation operating point is ' + str(wind_speed_op) + 'm/s')
            print('Results may not be as expected')

            # Set linear system to only linearization
            A = self.A_ops[:, :, 0]
            B = self.B_ops[:, :, 0]
            C = self.C_ops[:, :, 0]
            D = self.D_ops[:, :, 0]

            u_op = self.u_ops[:, 0]
            y_op = self.y_ops[:, 0]
            x_op = self.x_ops[:, 0]

        # form state space model  TODO: generalize using linear description strings
        P_op = co.StateSpace(A, B, C, D)
        if reduce_states:
            P_op = co.minreal(P_op, tol=1e-7, verbose=False)
        P_op.InputName = self.DescCntrlInpt
        P_op.OutputName = self.DescOutput

        # operating points dict
        ops = {}
        ops['u'] = u_op[self.ind_fast_inps]
        ops['uh'] = wind_speed_op
        ops['y'] = y_op[self.ind_fast_outs]
        ops['x'] = x_op

        return ops, P_op

    def add_control(self, lin_control_model):
        '''
        Add closed loop control to plant, augmenting P_op with LinearControlModel object

        inputs: lin_control_model   - instance of LinearControlModel object
                ops               - wind speed operating point

        OutputName: P_cl               - linear closed-loop plant

        '''

        # find pitch control operating point
        pitch_op = self.ops['u'][1]
        lin_control_model.pitch_control(pitch_op)

        lin_control_model.connect_elements()
        #    TODO: add torque control

        P_cl = connect_ml([self.P_op, lin_control_model.C_all], self.DescCntrlInpt, self.DescOutput)

        return P_cl

    def solve(self, disturbance, Plot=False, open_loop=True, controller={}, reduce_states=False):
        ''' 
        Run linear simulation of open-loop turbine model
        inputs: disturbance: dict containing
                    Time - vector of time indices
                    Wind - vector of wind speeds (usually rotor avg wind speed)
                Plot - plot solution?
                open_loop - (logical) run linearization in open loop or add closed-loop linear controllers
                controller (optional) - linear controller to add if open_loop = False

        outputs: OutList - list of output channels, mimicking nonlinear OpenFAST
                 OutData - array of output channels, mimicking nonlinear OpenFAST
        '''
        # Unpack disturbance
        tt = disturbance['Time']
        u_h = disturbance['Wind']

        # Get plant operating point
        self.ops, self.P_op = self.get_plant_op(u_h, reduce_states)

        if not open_loop:
            if isinstance(controller, LinearControlModel):
                P_op = self.add_control(controller)
            else:
                print('WARNING: controller not LinearControlModel() object')

        # linearize input (remove wind_speed_op)
        u_lin = np.zeros((len(self.DescCntrlInpt), len(tt)))

        indWind = self.DescCntrlInpt.index(
            'IfW Extended input: horizontal wind speed (steady/uniform wind), m/s')
        u_lin[indWind, :] = u_h - self.ops['uh']

        # linear solve
        _, y_lin, xx = co.forced_response(P_op, T=tt, U=u_lin)

        # Add back in operating points
        # Note that bld pitch was an input operating point that we are moving to the output operating point
        # TODO: designate CONTROL inputs that become outputs in closed-loop simulations
        y_op = self.ops['y']  # TODO: make blade pitch operationg point in radians
        u_op = self.ops['u']

        y = y_op.reshape(-1, 1) + y_lin
        u = u_op[0].reshape(-1, 1) + u_lin

        # plot, for debugging
        if Plot:
            chans = ['RtVAvgxh', 'GenSpeed']
            fig, ax = plt.subplots(len(chans), 1)
            for i, chan in enumerate(chans):
                ind = [chan in out for out in self.DescOutput].index(True)      # channel index
                ax[i].plot(tt, y[ind, :])
                ax[i].grid(True)
                ax[i].set_ylabel(chan)

                if i < len(chans):
                    ax[i].set_xticklabels([])
            plt.show()

        # Shorten output names from linearization output to one like level3 openfast output
        # This depends on how openfast sets up the linearization output names and may break if that is changed
        OutList = [out_name.split()[1][:-1] for out_name in P_op.OutputName]
        OutData_arr = y.T

        # Turn OutData into dict like in ROSCO_toolbox
        OutData = {}
        for i, out_chan in enumerate(OutList):
            OutData[out_chan] = OutData_arr[:, i]

        # Add time to OutData
        OutData['Time'] = tt

        return OutData, OutList, P_op

    def trim_system(self, rm_azimuth=True, rm_hydro=True, desInputs=[], desOutputs=[]):
        '''
        Trim states, inputs, and outputs, of system
        '''

        # Find states to remove
        hd_idx = []
        az_idx = []
        if rm_hydro:
            hd_idx = np.flatnonzero(np.core.defchararray.find(self.DescStates, 'HD') > -1).tolist()
        if rm_azimuth:
            azDesc = 'ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad'
            az_idx = np.flatnonzero(np.core.defchararray.find(self.DescStates, azDesc) > -1).tolist()

        # Define states to keep
        indStates = np.arange(len(self.DescStates))
        indStates = np.delete(indStates, hd_idx + az_idx).reshape(-1, 1)

        # Find inputs to keep
        indInputs = np.arange(0, len(self.DescCntrlInpt))
        if desInputs:
            inp_idx = []
            for inp in desInputs:
                inp_idx += np.flatnonzero(np.core.defchararray.find(self.DescCntrlInpt, inp) > -1).tolist()
            indInputs = indInputs[inp_idx]
        indInputs = indInputs.reshape(-1,1)

        # Find outputs to keep
        indOutputs = np.arange(0, len(self.DescOutput))
        if desOutputs:
            out_idx = []
            for outp in desOutputs:
                out_idx += np.flatnonzero(np.core.defchararray.find(self.DescOutput, outp) > -1).tolist()
            indOutputs = indOutputs[out_idx]
        indOutputs = indOutputs.reshape(-1,1)

        # Setup new state matrices
        nStates  = int(len(indStates))
        nInputs  = len(indInputs)
        nOutputs = len(indOutputs)
        A_nohd = np.zeros(( nStates, nStates, self.n_lin_cases))
        B_nohd = np.zeros(( nStates, nInputs, self.n_lin_cases))
        C_nohd = np.zeros(( nOutputs, nStates, self.n_lin_cases))
        D_nohd = np.zeros(( nOutputs, nInputs, self.n_lin_cases))

        # Remove states
        for case_idx in range(self.n_lin_cases):
            A_nohd[:,:,case_idx] = self.A_ops[:,:,case_idx][indStates, indStates.T].squeeze()
            if len(indInputs) == 1:
                B_nohd[:,:,case_idx] = self.B_ops[:,:,case_idx][indStates, indInputs.T]
            else:
                B_nohd[:,:,case_idx] = self.B_ops[:,:,case_idx][indStates, indInputs.T].squeeze()
            C_nohd[:,:,case_idx] = self.C_ops[:,:,case_idx][indOutputs, indStates.T].squeeze()
            if len(indInputs) == 1:
                D_nohd[:, :, case_idx] = self.D_ops[:, :, case_idx][indOutputs, indInputs.T]
            else:
                D_nohd[:, :, case_idx] = self.D_ops[:, :, case_idx][indOutputs, indInputs.T].squeeze()

        # Replace old state matrices
        self.A_ops = A_nohd
        self.B_ops = B_nohd
        self.C_ops = C_nohd
        self.D_ops = D_nohd

        # Trim Descriptions
        self.DescStates     = np.array(self.DescStates)[indStates].squeeze().tolist()  
        self.x_ops          = self.x_ops[indStates].squeeze()
        self.StateDerivOrder = np.array(self.StateDerivOrder)[indStates].squeeze().tolist()
        self.DescCntrlInpt  = np.array(self.DescCntrlInpt)[indInputs].squeeze().tolist()
        self.DescOutput     = np.array(self.DescOutput)[indOutputs].squeeze().tolist()


class LinearControlModel(object):
    ''' 
    Linear control models from ROSCO input files or ROSCO_Toolbox controller objects

    Control Models:
        - baseline pitch, function of blade pitch operating point?
        - floating feedback

    TODO:
        - PI torque
    
    '''

    def __init__(self, controller, fromDISCON_IN=False, DISCON_file=[], floating=True):
        # Set parameters here for now
        '''
        controller is object from ROSCO Toolbox
        fromDISON_IN if you want to skip that and read directly from DISCON_file object
        '''

        # Define input/output description strings
        self.DescPitch = 'ED Extended input: collective blade-pitch command, rad'
        self.DescGen = 'ED GenSpeed, (rpm)'
        self.DescTwr = 'ED TwrBsMyt, (kN-m)'
        self.DescAz = 'ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad'
        self.DescPltPitch = 'ED PtfmPitch, (deg)'
        self.DescNacIMUFA = 'ED NcIMURAys, (deg/s^2)'
        # TODO: add torque control signals

        if not fromDISCON_IN:

            # Pitch control parameters, gain scheduled
            self.PC_GS_angles = controller.pitch_op_pc
            self.PC_GS_KP = controller.pc_gain_schedule.Kp
            self.PC_GS_KI = controller.pc_gain_schedule.Ki

            # Floating Control parameters
            self.Fl_Kp = controller.Kp_float
            Fl_Bw, Fl_Damp = controller.turbine.ptfm_freq, 1.0
            fl_lpf = self.low_pass_filter(Fl_Bw, Fl_Damp)

            # Pitch Actuator parameters
            # self.PC_ActBw           = controller.turbine.pitch_act_bw
            self.PC_ActBw = 100  # 1.5708  # hard code until this is pulled into ROSCO

            # Gen Speed Filter parameters
            F_Gen_Freq = controller.turbine.bld_edgewise_freq * 1/4
            F_Gen_Damp = controller.F_LPFDamping

        else:
            # Will probably want to move this out when we have other methods for setting up controller

            # Pitch control parameters, gain scheduled
            self.PC_GS_angles = DISCON_file['PC_GS_angles']
            self.PC_GS_KP = DISCON_file['PC_GS_KP']
            self.PC_GS_KI = DISCON_file['PC_GS_KI']

            # Floating Control parameters
            self.Fl_Kp = DISCON_file['Fl_Kp']
            Fl_Bw, Fl_Damp = DISCON_file['F_FlCornerFreq']
            fl_lpf = self.low_pass_filter(Fl_Bw, Fl_Damp)

            # Pitch Actuator parameters
            self.PC_ActBw = 100  # DISCON_file['PC_ActBw']

            # Gen Speed Filter parameters
            F_Gen_Freq = DISCON_file['F_LPFCornerFreq']
            F_Gen_Damp = DISCON_file['F_LPFDamping']

        # Floating transfer function
        s = co.TransferFunction.s
        self.C_Fl = - fl_lpf / s * np.mean(self.Fl_Kp) * deg2rad(1)
        self.C_Fl = co.ss(self.C_Fl)
        self.C_Fl.InputName = self.DescNacIMUFA
        self.C_Fl.OutputName = 'Fl_Pitch'

        # pitch actuator model
        pitch_act = self.low_pass_filter(self.PC_ActBw, .707)
        self.pitch_act = co.ss(pitch_act)
        self.pitch_act.InputName = 'PitchCmd'
        self.pitch_act.OutputName = self.DescPitch

        # generator filter model
        self.F_Gen = self.low_pass_filter(F_Gen_Freq, F_Gen_Damp)
        self.F_Gen = co.ss(self.F_Gen)
        self.F_Gen.InputName = self.DescGen
        self.F_Gen.OutputName = 'GenSpeedF'

    def pitch_control(self, pitch_op):
        ''' Linear pitch controller, using pitch_op (rad) to determine PI gains

            output: C_PC - control system object with pi pitch controller
        '''

        f_p = sp.interpolate.interp1d(self.PC_GS_angles, self.PC_GS_KP)
        f_i = sp.interpolate.interp1d(self.PC_GS_angles, self.PC_GS_KI)

        kp = f_p(pitch_op)
        ki = f_i(pitch_op)

        s = co.TransferFunction.s
        self.C_PC = -(kp + ki/s) * rpm2radps(1)
        self.C_PC = co.ss(self.C_PC)
        self.C_PC.InputName = 'GenSpeedF'
        self.C_PC.OutputName = 'PC_Pitch'

    def connect_elements(self):
        '''
        Connect various control elements
        required: C_PC, C_Fl, F_Gen, pitch_act
        '''
        # sum block
        S = self.sum_block(['PC_Pitch', 'Fl_Pitch'], 'PitchCmd')

        # Have to connect everything...

        # control modules
        mods = [self.C_PC, self.C_Fl, S, self.F_Gen, self.pitch_act]
        # mods = [self.C_PC,self.C_Fl,S,self.pitch_act]
        # mods = [self.C_PC]

        inVNames = [self.DescGen, self.DescNacIMUFA]
        outVNames = [self.DescPitch]

        self.C_all = connect_ml(mods, inVNames, outVNames)

    def low_pass_filter(self, omega, zeta, order=2):
        '''
            Returns low pass filter control object
        '''

        if order == 2:
            lpf = co.tf([omega**2], [1, 2*zeta*omega, omega**2])
        elif order > 2:
            [b, a] = sp.signal.butter(order, omega, analog=True)
            lpf = co.tf(b, a)
        else:
            lpf = co.tf(omega, [1, omega])

        return lpf

    def sum_block(self, InputName, OutputName):
        '''
            Similar to sumblk() in matlab

            InputName: list of input signals, corresponding to control object I/O names
            OutputName: OutputName signal, corresponding to some control object I/O name


        '''
        S = co.ss(0, [0]*len(InputName), 0, [1]*len(InputName))      # 2D example

        # S = co.tf(S)
        S.InputName = InputName
        S.OutputName = OutputName
        S.dt = 0

        return S


# helper functions
def run_mbc3(fnames):
    '''
    Helper function to run mbc3
    '''
    print('Loading linearizations from:', ''.join(fnames[0].split('.')[:-2]))
    MBC, matData, FAST_linData = mbc.fx_mbc3(fnames, verbose=False)

    return MBC, matData, FAST_linData

def connect_ml(mods, inputs, outputs):
    ''' 
    Connects models like the matlab function does

    inputs: mods  - list of ss models, should have InputName and OutputName fields
            inputs - list of inputs of connected system
            outputs - list of ouptuts of connected system

    output: sys  - connected ss model

    '''

    # Input checking - all timesteps should be the same as the first one
    if mods[0].dt is None:
        mods[0].dt = 0

    dt_main = mods[0].dt

    for mod in mods:
        # timesteps
        if mod.dt is None:
            mod.dt = 0
        elif mod.dt != dt_main:
            print('Some systems have different dts')

        if not hasattr(mod, 'InputName'):
            print('WARNING: missing InputName')

        if not hasattr(mod, 'OutputName'):
            print('WARNING: missing OutputName')

    # append sequentially
    sys = co.ss([], [], [], [])    # init empty system
    sys.dt = 0

    for mod in mods:
        sys = co.append(sys, mod)

    # Set of input/output names
    InNames = []
    for mod in mods:
        if isinstance(mod.InputName, list):
            for InName in mod.InputName:
                InNames.append(InName)
        else:
            InNames.append(mod.InputName)

    OutNames = []
    for mod in mods:
        if isinstance(mod.OutputName, list):
            for OutName in mod.OutputName:
                OutNames.append(OutName)
        else:
            OutNames.append(mod.OutputName)

    # init interconnection matrix
    Q = np.zeros((len(InNames), 2), dtype=int)

    # assign input list into sequential indices
    Q[:, 0] = np.arange(1, len(InNames)+1)

    # assign output list to input indices
    for input_name in InNames:
        if input_name in OutNames:
            Q[InNames.index(input_name), 1] = OutNames.index(input_name)+1

    # Input/Output indices, TODO: better error catching here
    inV = [InNames.index(in_v_name) + 1 for in_v_name in inputs]
    outV = [OutNames.index(out_v_name) + 1 for out_v_name in outputs]

    # Connect...finally!
    # Q = np.array(([1,0,],[2,5],[3,1])).tolist()
    sys = co.connect(sys, Q, inV, outV)
    sys.InputName = inputs
    sys.OutputName = outputs

    return sys

def deg2rad(d):
    return d * np.pi / 180


def rad2deg(r):
    return r * 180 / np.pi


def rpm2radps(r):
    return r * 2 * np.pi / 60


def radps2rpm(r):
    return r * 60 / 2 / np.pi
