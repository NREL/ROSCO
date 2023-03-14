'''
----------- 24_rotor_position_control --------------
Run ROSCO with rotor position control
-------------------------------------

Run a steady simulation, use the azimuth output as an input to the next steady simulation, with different ICs 

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.controller import OpenLoopControl
import numpy as np
import pandas as pd
import pickle

RPS2RPM          = 9.5492966
GB_RATIO = 111.7

#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir         = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))

def read_scada(scada_file,mbc=False):
    # try:
    df = pd.read_csv(scada_file)


    df.rename(columns = {
        'Time (s)':'Time',
        'Gen Power (kW)': 'GenPwr',
        'Root Bending Flap 1 (kN-m)': 'RootMyb1',
        'Root Bending Flap 2 (kN-m)': 'RootMyb2',
        'Root Bending Flap 3 (kN-m)': 'RootMyb3',
        'Rotor Speed (rpm)': 'RotSpd',
        'Gen Torque (kN-m)':'GenTq',
        'Azimuth (deg.)':'Azimuth',
        'Pitch 1 (deg.)':'BldPitch1',
        'Pitch 2 (deg.)':'BldPitch2',
        'Pitch 3 (deg.)':'BldPitch3',
        'Yaw Misalignment (deg.)': 'YawMisalign',
    },inplace=True)

    # Calculate generator torque 

    GenSpd = df.RotSpd / RPS2RPM * GB_RATIO

    GenTq = df.GenPwr/GenSpd

    df['GenTq'] = GenTq

    # remove pitch offset, assume BldPitch1 is true for now

    offset_2 = np.mean(df.BldPitch1 - df.BldPitch2)
    offset_3 = np.mean(df.BldPitch1 - df.BldPitch3)

    df.BldPitch2 = df.BldPitch2 + offset_2
    df.BldPitch3 = df.BldPitch3 + offset_3

    if mbc:
        # MBC Transform
        n = 1
        pi = np.pi

        phi2 = 2/3 * pi
        phi3 = 4/3 * pi

        M_tilt = np.zeros(len(df))
        M_yaw = np.zeros(len(df))

        B_tilt = np.zeros(len(df))
        B_yaw = np.zeros(len(df))

        for i, d in enumerate(df.iterrows()):
            d = d[1]

            az = np.radians(d.Azimuth)

            M = [d.RootMyb1,d.RootMyb2,d.RootMyb3]


            B = np.array([d.BldPitch1,d.BldPitch2,d.BldPitch3])
            B = B - np.mean(B)


            M_tilt[i] = 2/3 * (np.cos(n *az) * M[0] + np.cos(n*(az - phi2)) * M[1] + np.cos(n*(az - phi3)) * M[2])
            M_yaw[i] =  2/3 * (np.sin(n *az) * M[0] + np.sin(n*(az - phi2)) * M[1] + np.sin(n*(az - phi3)) * M[2]) 

            B_tilt[i] = 2/3 * (np.cos(n *az) * B[0] + np.cos(n*(az - phi2)) * B[1] + np.cos(n*(az - phi3)) * B[2])
            B_yaw[i] =  2/3 * (np.sin(n *az) * B[0] + np.sin(n*(az - phi2)) * B[1] + np.sin(n*(az - phi3)) * B[2]) 


        df['RootMTilt'] = M_tilt
        df['RootMYaw'] = M_yaw 

        df['B_Tilt'] = B_tilt
        df['B_Yaw'] = B_yaw 


        # Compute summary data

        # df_summ = df_summ.append({
        #     'RootMyb1_std': df.RootMyb1.std(),
        #     'IPC_act': np.sum(df['B_Tilt'] > 0.75) / len(df['B_Tilt']),
        # },ignore_index=True)
        
        
    return df

def main():

    OPENFAST = True


    # Ensure external control paths are okay
    wind_bin = 891
    parameter_filename = '/Users/dzalkind/Projects/RAAW/RAAW_OpenFAST/ROSCO/RAAW_rosco_BD.yaml'
    run_dir = os.path.join(example_out_dir,'24_rotor_position_control_0')
    gains = -1800 * np.array([12,1.2,120])
    os.makedirs(run_dir,exist_ok=True)

    iterate = False
    iterate_file = os.path.join(example_out_dir,'19_RotPos_SCADA_20/rpc/turb_bts/base/rpc_0.out')

    # Case input for RotSpeed IC
    case_inputs = {}
    case_inputs[("ElastoDyn","RotSpeed")]    = {'vals':[5], 'group':0}
    case_inputs[("ElastoDyn","PtfmSgDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmSwDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmHvDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmRDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmPDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmYDOF")]    = {'vals':['False'], 'group':0}
    
    # Steady simualtion with initial RotSpeed of 5 rpm
    r = run_FAST_ROSCO()
    r.wind_case_fcn = cl.turb_bts
    r.wind_case_opts    = {
        'TMax': 600,
        'wind_filenames': ['/Users/dzalkind/Tools/WEIS-1/outputs/02_RAAW_TurbOff/wind/IEA15_NTM_U8.000000_Seed123.0.bts']
        }
    r.save_dir      = run_dir
    r.case_inputs   = case_inputs
    r.run_FAST()

    # Gather azimuth, blade p
    # itch, generator torque output
    if OPENFAST:
        op = output_processing.output_processing()
        # fast_out = op.load_fast_out(os.path.join(example_out_dir,'19_RotPos_Turb_0/IEA15MW/turb_bts/base/IEA15MW_0.outb'), tmin=0)
        bl_out = '/Users/dzalkind/Tools/ROSCO2/outputs/RPC_Sweeps/Baseline_Cases/RAAW_rosco_BD/turb_bts/base/RAAW_rosco_BD_0.outb'
        fast_out = op.load_fast_out(bl_out)

        olc = OpenLoopControl()
        olc.ol_timeseries['time'] = fast_out[0]['Time']
        olc.ol_timeseries['blade_pitch1'] = np.radians(fast_out[0]['BldPitch1'])
        olc.ol_timeseries['blade_pitch2'] = np.radians(fast_out[0]['BldPitch2'])
        olc.ol_timeseries['blade_pitch3'] = np.radians(fast_out[0]['BldPitch3'])
        olc.ol_timeseries['generator_torque'] = fast_out[0]['GenTq'] * 1000
        olc.ol_timeseries['azimuth'] = np.radians(fast_out[0]['Azimuth'])

        RotSpeed_0 = fast_out[0]['RotSpeed'][0]
        Azimuth_0 = fast_out[0]['Azimuth'][0]
        BldPitch_0 = fast_out[0]['BldPitch1'][0]
    else:
        # print('here')
        scada_file = f'/Users/dzalkind/Box/EXT - RAAW data sharing/GE Proprietary/Data Assimilation (Mid-Fidelity)/Historical Data/v31/_Diagnostics/_GeneralDiagnostic/_TurbineConditions_Timeseries_Bin{wind_bin}.csv'
        df = read_scada(scada_file)
                
        # Simple integrate
        Az_unwrap = np.unwrap(np.radians(df.Azimuth))
        tt = df.Time
        omega = df.RotSpd / 9.5492966

        int_RotSpeed = np.empty(len(omega))
        for i_t in range(len(omega)):
            int_RotSpeed[i_t] = np.trapz(omega[:i_t],tt[:i_t]) + Az_unwrap[0]

        olc = OpenLoopControl()
        olc.ol_timeseries['time'] = tt.to_numpy()
        olc.ol_timeseries['blade_pitch1'] = np.radians(df.BldPitch1).to_numpy()
        olc.ol_timeseries['blade_pitch2'] = np.radians(df.BldPitch2).to_numpy()
        olc.ol_timeseries['blade_pitch3'] = np.radians(df.BldPitch3).to_numpy()
        olc.ol_timeseries['generator_torque'] = df.GenTq * 1000
        olc.ol_timeseries['azimuth'] = int_RotSpeed

        # ICs
        RotSpeed_0 = df.RotSpd[0]
        Azimuth_0 = np.degrees(int_RotSpeed[0])

        if iterate:  # iterate
            op = output_processing.output_processing()
            fast_out = op.load_fast_out(iterate_file, tmin=0)
            olc.ol_timeseries['generator_torque'] = np.interp(olc.ol_timeseries['time'],fast_out[0]['Time'],fast_out[0]['GenTq'] * 1000)

    # Save olc
    olc.RotSpeed_0 = RotSpeed_0
    olc.Azimuth_0 = Azimuth_0
    olc.BldPitch_0 = BldPitch_0
    case_ind = bl_out.split('.out')[0][-1]
    case_dir = os.path.split(bl_out)[0]
    with open(os.path.join(case_dir,f'olc_{case_ind}.p'),'wb') as f:
        pickle.dump(olc,f)
    
    # set up control_params for next run
    open_loop = olc.write_input(os.path.join(run_dir,'ol_input.dat'))
    controller_params = {}
    controller_params['open_loop'] = open_loop
    controller_params['OL_Mode'] = 2
    controller_params['PA_Mode'] = 0
    controller_params['DISCON'] = {}
    controller_params['DISCON']['RP_Gains'] = gains
    controller_params['DISCON']['PC_MinPit'] = np.radians(-20)


    # run again with slower IC and rotor position control
    # case_inputs[("ElastoDyn","RotSpeed")]    = {'vals':[4], 'group':0}
    r.case_inputs   = case_inputs
    r.base_name     = 'rpc'
    r.tuning_yaml   = parameter_filename
    r.wind_case_opts    = {
        'TMax': 600,
        # 'wind_filenames': [f'/Users/dzalkind/Projects/RAAW/RAAW_OpenFAST/wind/bin{wind_bin}-turbsim.bts'],
        'wind_filenames': ['/Users/dzalkind/Downloads/RAAW_0_NTM_U9.000000_Seed533103612.0.bts'],
        }
    # Set initial conditions
    r.case_inputs = {}
    r.case_inputs[("ElastoDyn","RotSpeed")]      = {'vals':[RotSpeed_0], 'group':0}
    r.case_inputs[("ElastoDyn","Azimuth")]      = {'vals':[Azimuth_0], 'group':0}
    r.case_inputs[("ServoDyn","Ptch_Cntrl")]      = {'vals':[1], 'group':0}

    r.controller_params = controller_params
    r.openfast_exe = '/Users/dzalkind/opt/anaconda3/envs/rosco-env2/bin/openfast'
    # r.run_FAST()



if __name__=="__main__":
    main()