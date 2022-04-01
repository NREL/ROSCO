import os, yaml
import numpy as np

from ROSCO_toolbox.ofTools.case_gen.CaseGen_General import CaseGen_General
from ROSCO_toolbox.ofTools.case_gen.CaseGen_IEC import CaseGen_IEC
from ROSCO_toolbox.ofTools.case_gen.HH_WindFile import HH_StepFile

# ROSCO 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import utilities as ROSCO_utilities

from ROSCO_toolbox.inputs.validation import load_rosco_yaml

# Globals
this_dir        = os.path.dirname(os.path.abspath(__file__))
tune_case_dir   = os.path.realpath(os.path.join(this_dir,'../../../Tune_Cases'))

def find_max_group(case_inputs):
    max_group = 0
    for ci in case_inputs:
        max_group = np.max([case_inputs[ci]['group'],max_group])
    return max_group


def set_channels():
    channels = {}
    for var in ["TipDxc1", "TipDyc1", "TipDzc1", "TipDxb1", "TipDyb1", "TipDxc2", "TipDyc2", \
         "TipDzc2", "TipDxb2", "TipDyb2", "TipDxc3", "TipDyc3", "TipDzc3", "TipDxb3", "TipDyb3", \
             "RootMxc1", "RootMyc1", "RootMzc1", "RootMxb1", "RootMyb1", "RootMxc2", "RootMyc2", \
                 "RootMzc2", "RootMxb2", "RootMyb2", "RootMxc3", "RootMyc3", "RootMzc3", "RootMxb3",\
                      "RootMyb3", "TwrBsMxt", "TwrBsMyt", "TwrBsMzt", "GenPwr", "GenTq", "RotThrust",\
                           "RtAeroCp", "RtAeroCt", "RotSpeed", "BldPitch1", "TTDspSS", "TTDspFA", \
                               "NcIMUTAxs", "NcIMUTAys", "NcIMUTAzs", "NcIMURAxs", "NcIMURAys", "NcIMURAzs", \
                                "NacYaw", "Wind1VelX", "Wind1VelY", "Wind1VelZ", "LSSTipMxa","LSSTipMya",\
                                   "LSSTipMza","LSSTipMxs","LSSTipMys","LSSTipMzs","LSShftFys","LSShftFzs", \
                                       "TipRDxr", "TipRDyr", "TipRDzr","RtVAvgxh"]:
        channels[var] = True
    return channels


def load_tuning_yaml(tuning_yaml):
    # Load yaml file, this could be an object...
    inps = load_rosco_yaml(tuning_yaml)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(tune_case_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
    turbine.load_from_fast(path_params['FAST_InputFile'], \
        os.path.join(tune_case_dir,path_params['FAST_directory']), \
        dev_branch=True,rot_source='txt',\
        txt_filename=cp_filename)

    return turbine, controller, cp_filename


##############################################################################################
#
#   Wind input cases
#
##############################################################################################

def power_curve(run_dir):
    # Constant wind speed, multiple wind speeds, define below

    # Runtime
    T_max   = 400.

    # Run conditions
    U = np.arange(4,14.5,.5).tolist()
    U = np.linspace(9.5,12,num=16)


    case_inputs = {}
    # simulation settings
    case_inputs[("Fst","TMax")] = {'vals':[T_max], 'group':0}

    # DOFs
    if False:
        case_inputs[("ElastoDyn","YawDOF")]      = {'vals':['True'], 'group':0}
        case_inputs[("ElastoDyn","FlapDOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","FlapDOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","EdgeDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","DrTrDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","GenDOF")]      = {'vals':['True'], 'group':0} 
        case_inputs[("ElastoDyn","TwFADOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwFADOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwSSDOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwSSDOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmSgDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmHvDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmPDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmSwDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmRDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmYDOF")]     = {'vals':['False'], 'group':0}
    
    # wind inflow
    case_inputs[("InflowWind","WindType")] = {'vals':[1], 'group':0}
    case_inputs[("InflowWind","HWindSpeed")] = {'vals':U, 'group':1}

    # Stop Generator from Turning Off
    case_inputs[('ServoDyn', 'GenTiStr')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'GenTiStp')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'SpdGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'TimGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'GenModel')] = {'vals': [1], 'group': 0}
    

    # AeroDyn
    case_inputs[("AeroDyn15", "WakeMod")] = {'vals': [1], 'group': 0}
    case_inputs[("AeroDyn15", "AFAeroMod")] = {'vals': [2], 'group': 0}
    case_inputs[("AeroDyn15", "TwrPotent")] = {'vals': [0], 'group': 0}
    case_inputs[("AeroDyn15", "TwrShadow")] = {'vals': ['False'], 'group': 0}
    case_inputs[("AeroDyn15", "TwrAero")] = {'vals': ['False'], 'group': 0}
    case_inputs[("AeroDyn15", "SkewMod")] = {'vals': [1], 'group': 0}
    case_inputs[("AeroDyn15", "TipLoss")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "HubLoss")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "TanInd")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "AIDrag")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "TIDrag")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "IndToler")] = {'vals': [1.e-5], 'group': 0}
    case_inputs[("AeroDyn15", "MaxIter")] = {'vals': [5000], 'group': 0}
    case_inputs[("AeroDyn15", "UseBlCm")] = {'vals': ['True'], 'group': 0}

    return case_inputs

    # # Controller
    # if rosco_dll:
    #     # Need to update this to ROSCO with power control!!!
    #     case_inputs[("ServoDyn","DLL_FileName")] = {'vals':[rosco_dll], 'group':0}

    # # Control (DISCON) Inputs
    # discon_vt = ROSCO_utilities.read_DISCON(discon_file)
    # for discon_input in discon_vt:
    #     case_inputs[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

    # from weis.aeroelasticse.CaseGen_General import CaseGen_General
    # case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=runDir, namebase=namebase)

    # channels = set_channels()

    return case_list, case_name_list, channels

def simp_step(run_dir):
    # Set up cases for FIW-JIP project
    # 3.x in controller tuning register

    # Default Runtime
    T_max   = 300.

    # Step Wind Setup

    # Make Default step wind object
    hh_step = HH_StepFile()
    hh_step.t_max = T_max
    hh_step.t_step = 150
    hh_step.wind_directory = run_dir

    # Run conditions
    U_start     = [16]
    U_end       = [17]
    step_wind_files = []

    for u_s,u_e in zip(U_start,U_end):
        # Make Step
        hh_step.u_start = u_s
        hh_step.u_end   = u_e
        hh_step.update()
        hh_step.write()

        step_wind_files.append(hh_step.filename)

    case_inputs = {}
    # simulation settings
    case_inputs[("Fst","TMax")] = {'vals':[T_max], 'group':0}
    case_inputs[("Fst","OutFileFmt")]        = {'vals':[2], 'group':0}
    # case_inputs[("Fst","DT")]        = {'vals':[1/80], 'group':0}

    # DOFs
    # case_inputs[("ElastoDyn","YawDOF")]      = {'vals':['True'], 'group':0}
    # case_inputs[("ElastoDyn","FlapDOF1")]    = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","FlapDOF2")]    = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","EdgeDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","DrTrDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","GenDOF")]      = {'vals':['True'], 'group':0} 
    # case_inputs[("ElastoDyn","TwFADOF1")]    = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","TwFADOF2")]    = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","TwSSDOF1")]    = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","TwSSDOF2")]    = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","PtfmSgDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","PtfmHvDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","PtfmPDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","PtfmSwDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","PtfmRDOF")]     = {'vals':['False'], 'group':0}
    # case_inputs[("ElastoDyn","PtfmYDOF")]     = {'vals':['False'], 'group':0}
    
    # wind inflow
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","Filename_Uni")] = {'vals':step_wind_files, 'group':1}


    # Stop Generator from Turning Off
    case_inputs[('ServoDyn', 'GenTiStr')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'GenTiStp')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'SpdGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'TimGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'GenModel')] = {'vals': [1], 'group': 0}
    

    # AeroDyn
    case_inputs[("AeroDyn15", "WakeMod")] = {'vals': [1], 'group': 0}
    case_inputs[("AeroDyn15", "AFAeroMod")] = {'vals': [2], 'group': 0}
    case_inputs[("AeroDyn15", "TwrPotent")] = {'vals': [0], 'group': 0}
    case_inputs[("AeroDyn15", "TwrShadow")] = {'vals': ['False'], 'group': 0}
    case_inputs[("AeroDyn15", "TwrAero")] = {'vals': ['False'], 'group': 0}
    case_inputs[("AeroDyn15", "SkewMod")] = {'vals': [1], 'group': 0}
    case_inputs[("AeroDyn15", "TipLoss")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "HubLoss")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "TanInd")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "AIDrag")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "TIDrag")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "IndToler")] = {'vals': [1.e-5], 'group': 0}
    case_inputs[("AeroDyn15", "MaxIter")] = {'vals': [5000], 'group': 0}
    case_inputs[("AeroDyn15", "UseBlCm")] = {'vals': ['True'], 'group': 0}



    # # Tune Floating Feedback Gain
    # if tune == 'fl_gain':
    #     case_inputs[('DISCON_in','Fl_Kp')] = {'vals': np.linspace(0,-18,6,endpoint=True).tolist(), 'group': 2}

    # elif tune == 'fl_phase':
    #     case_inputs[('DISCON_in','Fl_Kp')] = {'vals': 8*[-25], 'group': 2}
    #     case_inputs[('DISCON_in','F_FlCornerFreq')] = {'vals': 8*[0.300], 'group': 2}
    #     case_inputs[('DISCON_in','F_FlHighPassFreq')] = {'vals':[0.001,0.005,0.010,0.020,0.030,0.042,0.060,0.100], 'group': 2}
    #     case_inputs[('meta','Fl_Phase')] = {'vals':8*[-50],'group':2}

    # elif tune == 'pc_mode':
    #     # define omega, zeta
    #     omega = np.linspace(.05,.25,8,endpoint=True).tolist()
    #     zeta  = np.linspace(1,3,3,endpoint=True).tolist()
        
    #     control_case_inputs = sweep_pc_mode(omega,zeta)
    #     case_inputs.update(control_case_inputs)


    # elif tune == 'ps_perc':
    #     # Set sweep limits here
    #     ps_perc = np.linspace(.75,1,num=8,endpoint=True).tolist()
        
    #     # load default params          
    #     weis_dir            = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    #     control_param_yaml  = os.path.join(weis_dir,'examples/OpenFAST_models/CT15MW-spar/ServoData/IEA15MW-CT-spar.yaml')
    #     inps                = yaml.safe_load(open(control_param_yaml))
    #     path_params         = inps['path_params']
    #     turbine_params      = inps['turbine_params']
    #     controller_params   = inps['controller_params']

    #     # make default controller, turbine objects for ROSCO_toolbox
    #     turbine             = ROSCO_turbine.Turbine(turbine_params)
    #     turbine.load_from_fast( path_params['FAST_InputFile'],path_params['FAST_directory'], dev_branch=True)

    #     controller          = ROSCO_controller.Controller(controller_params)

    #     # tune default controller
    #     controller.tune_controller(turbine)

    #     # Loop through and make min pitch tables
    #     ps_ws = []
    #     ps_mp = []
    #     m_ps  = []  # flattened (omega,zeta) pairs
    #     for p in ps_perc:
    #         controller.ps_percent = p
    #         controller.tune_controller(turbine)
    #         m_ps.append(controller.ps_min_bld_pitch)

    #     # add control gains to case_list
    #     case_inputs[('meta','ps_perc')]          = {'vals': ps_perc, 'group': 2}
    #     case_inputs[('DISCON_in', 'PS_BldPitchMin')] = {'vals': m_ps, 'group': 2}

    # elif tune == 'max_tq':
    #     case_inputs[('DISCON_in','VS_MaxTq')] = {'vals': [19624046.66639, 1.5*19624046.66639], 'group': 3}

    # elif tune == 'yaw':
    #     case_inputs[('ElastoDyn','NacYaw')]     = {'vals': [-10,0,10], 'group': 3}



    return case_inputs


def steps(discon_file,runDir, namebase,rosco_dll=''):
    # Set up cases for FIW-JIP project
    # 3.x in controller tuning register

    # Default Runtime
    T_max   = 800.

    # Step Wind Setup

    # Make Default step wind object
    hh_step = HH_StepFile()
    hh_step.t_max = T_max
    hh_step.t_step = 400
    hh_step.wind_directory = runDir

    # Run conditions
    U = np.arange(4,24,1).tolist()
    step_wind_files = []

    for u in U:
        # Step up
        hh_step.u_start = u
        hh_step.u_end   = u+1
        hh_step.update()
        hh_step.write()

        step_wind_files.append(hh_step.filename)

        # Step down
        hh_step.u_start = u+1
        hh_step.u_end   = u
        hh_step.update()
        hh_step.write()

        step_wind_files.append(hh_step.filename)

    case_inputs = {}
    # simulation settings
    case_inputs[("Fst","TMax")] = {'vals':[T_max], 'group':0}
    case_inputs[("Fst","OutFileFmt")]        = {'vals':[2], 'group':0}

    # DOFs
    if True:
        case_inputs[("ElastoDyn","YawDOF")]      = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","FlapDOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","FlapDOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","EdgeDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","DrTrDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","GenDOF")]      = {'vals':['True'], 'group':0} 
        case_inputs[("ElastoDyn","TwFADOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwFADOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwSSDOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwSSDOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmSgDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmHvDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmPDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmSwDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmRDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmYDOF")]     = {'vals':['False'], 'group':0}
    
    # wind inflow
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","Filename")] = {'vals':step_wind_files, 'group':1}


    # Stop Generator from Turning Off
    case_inputs[('ServoDyn', 'GenTiStr')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'GenTiStp')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'SpdGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'TimGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'GenModel')] = {'vals': [1], 'group': 0}
    

    # AeroDyn
    case_inputs[("AeroDyn15", "WakeMod")] = {'vals': [1], 'group': 0}
    case_inputs[("AeroDyn15", "AFAeroMod")] = {'vals': [2], 'group': 0}
    case_inputs[("AeroDyn15", "TwrPotent")] = {'vals': [0], 'group': 0}
    case_inputs[("AeroDyn15", "TwrShadow")] = {'vals': ['False'], 'group': 0}
    case_inputs[("AeroDyn15", "TwrAero")] = {'vals': ['False'], 'group': 0}
    case_inputs[("AeroDyn15", "SkewMod")] = {'vals': [1], 'group': 0}
    case_inputs[("AeroDyn15", "TipLoss")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "HubLoss")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "TanInd")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "AIDrag")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "TIDrag")] = {'vals': ['True'], 'group': 0}
    case_inputs[("AeroDyn15", "IndToler")] = {'vals': [1.e-5], 'group': 0}
    case_inputs[("AeroDyn15", "MaxIter")] = {'vals': [5000], 'group': 0}
    case_inputs[("AeroDyn15", "UseBlCm")] = {'vals': ['True'], 'group': 0}

    # Controller
    if rosco_dll:
        # Need to update this to ROSCO with power control!!!
        case_inputs[("ServoDyn","DLL_FileName")] = {'vals':[rosco_dll], 'group':0}

    # Control (DISCON) Inputs
    discon_vt = ROSCO_utilities.read_DISCON(discon_file)
    for discon_input in discon_vt:
        case_inputs[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

    from weis.aeroelasticse.CaseGen_General import CaseGen_General
    case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=runDir, namebase=namebase)

    channels = set_channels()

    return case_list, case_name_list, channels


def sweep_pc_mode(cont_yaml,omega=np.linspace(.05,.35,8,endpoint=True).tolist(),zeta=[1.5],group=2):
    
    
    inps                = yaml.safe_load(open(cont_yaml))
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # make default controller, turbine objects for ROSCO_toolbox
    turbine             = ROSCO_turbine.Turbine(turbine_params)
    turbine.load_from_fast( path_params['FAST_InputFile'],path_params['FAST_directory'], dev_branch=True)

    controller          = ROSCO_controller.Controller(controller_params)

    # tune default controller
    controller.tune_controller(turbine)

    # check if inputs are lists
    if not isinstance(omega,list):
        omega = [omega]
    if not isinstance(zeta,list):
        zeta = [zeta]

    # Loop through and make PI gains
    pc_kp = []
    pc_ki = []
    m_omega = []  # flattened (omega,zeta) pairs
    m_zeta = []  # flattened (omega,zeta) pairs
    for o in omega:
        for z in zeta:
            controller.omega_pc = o
            controller.zeta_pc  = z
            controller.tune_controller(turbine)
            pc_kp.append(controller.pc_gain_schedule.Kp.tolist())
            pc_ki.append(controller.pc_gain_schedule.Ki.tolist())
            m_omega.append(o)
            m_zeta.append(z)

    # add control gains to case_list
    case_inputs = {}
    case_inputs[('meta','omega')]          = {'vals': m_omega, 'group': group}
    case_inputs[('meta','zeta')]          = {'vals': m_zeta, 'group': group}
    case_inputs[('DISCON_in', 'PC_GS_KP')] = {'vals': pc_kp, 'group': group}
    case_inputs[('DISCON_in', 'PC_GS_KI')] = {'vals': pc_ki, 'group': group}

    return case_inputs

# Control sweep functions
# function(controller,turbine,start_group)


def sweep_rated_torque(tuning_yaml,start_group):

    # Sweep multiplier of original rated torque
    multipliers = np.linspace(1,1.5,5)


    # Load yaml file 
    inps = load_rosco_yaml(tuning_yaml)
    path_params         = inps['path_params']
    controller_params   = inps['controller_params']

    var_trees = []
    for m in multipliers:
        
        turbine_params      = inps['turbine_params'].copy()
        turbine_params['rated_power'] *= m

        # Instantiate turbine, controller, and file processing classes
        turbine         = ROSCO_turbine.Turbine(turbine_params)
        controller      = ROSCO_controller.Controller(controller_params)

        # Load turbine data from OpenFAST and rotor performance text file
        cp_filename = os.path.join(tune_case_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
        turbine.load_from_fast(path_params['FAST_InputFile'], \
            os.path.join(tune_case_dir,path_params['FAST_directory']), \
            dev_branch=True,rot_source='txt',\
            txt_filename=cp_filename)

        controller.tune_controller(turbine)
        discon_vt = ROSCO_utilities.DISCON_dict(turbine, controller, txt_filename=cp_filename)            
        var_trees.append(discon_vt.copy())

    # Translate array of dicts into dict of arrays
    discon_array = {}
    for var in var_trees[0]:
        discon_array[var] = []

    for vt in var_trees:
        for var in vt:
            discon_array[var].append(vt[var])
            
    
    case_inputs_control = {}
    for discon_input in discon_array:
        case_inputs_control[('DISCON_in',discon_input)] = {'vals': discon_array[discon_input], 'group': start_group}

    return case_inputs_control
 





