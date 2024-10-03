"""
17a_zeromq_simple
--------------------
Run ROSCO using the ROSCO toolbox control interface and execute communication with ZeroMQ.

A demonstrator for ZeroMQ communication. Instead of using ROSCO with with control interface, 
one could call ROSCO from OpenFAST, and communicate with ZeroMQ through that.
this_dir"""


import os
import matplotlib.pyplot as plt
from rosco import discon_lib_path
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.control_interface import wfc_zmq_server
from rosco.toolbox import sim as ROSCO_sim
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox.ofTools.fast_io import output_processing
import numpy as np
import multiprocessing as mp

# Directories
THIS_DIR            = os.path.dirname(os.path.abspath(__file__))
EXAMPLE_OUT_DIR     = os.path.join(THIS_DIR,'examples_out')
os.makedirs(EXAMPLE_OUT_DIR,exist_ok=True)

# Controller parameters
TIME_CHECK = 30
DESIRED_YAW_OFFSET = 20
DESIRED_PITCH_OFFSET = np.deg2rad(2) * np.sin(0.1 * TIME_CHECK) + np.deg2rad(2)

def main():
    logfile = os.path.join(EXAMPLE_OUT_DIR,os.path.splitext(os.path.basename(__file__))[0]+'.log')
    p1 = mp.Process(target=run_zmq,args=(logfile,))
    p1.start()
    p2 = mp.Process(target=sim_rosco)
    p2.start()
    p1.join()
    p2.join()


def run_zmq(logfile=None):
    # Start the server at the following address
    network_address = "tcp://*:5555"
    server = wfc_zmq_server(network_address, timeout=60.0, verbose=False, logfile=logfile)

    # Provide the wind farm control algorithm as the wfc_controller method of the server
    server.wfc_controller = wfc_controller

    # Run the server to receive measurements and send setpoints
    server.runserver()
    
def wfc_controller(id,current_time,measurements):
    
    if current_time <= 10.0:
        yaw_setpoint = 0.0
    else:
        yaw_setpoint = DESIRED_YAW_OFFSET

    # Pitch offset
    if current_time >= 10.0:
        col_pitch_command = np.deg2rad(2) * np.sin(0.1 * current_time) + np.deg2rad(2) # Implement dynamic induction control
    else:
        col_pitch_command = 0.0

    # Send new setpoints back to ROSCO
    setpoints = {}
    setpoints['ZMQ_TorqueOffset'] = 0.0
    setpoints['ZMQ_YawOffset'] = yaw_setpoint
    setpoints['ZMQ_PitOffset(1)'] = col_pitch_command
    setpoints['ZMQ_PitOffset(2)'] = col_pitch_command
    setpoints['ZMQ_PitOffset(3)'] = col_pitch_command
    return setpoints

def sim_rosco():
    # Load yaml file
    THIS_DIR = os.path.dirname(os.path.abspath(__file__))
    tune_dir = os.path.join(THIS_DIR, 'Tune_Cases')
    parameter_filename = os.path.join(tune_dir, 'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']

    # Enable ZeroMQ & yaw control
    controller_params['Y_ControlMode'] = 1
    controller_params['ZMQ_Mode'] = 1
    controller_params['ZMQ_UpdatePeriod'] = 0.025

    # # Load turbine model from saved pickle
    turbine = ROSCO_turbine.Turbine
    turbine = turbine.load(os.path.join(EXAMPLE_OUT_DIR, '01_NREL5MW_saved.p'))

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(
        tune_dir, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir, path_params['FAST_directory']),
        rot_source='txt', txt_filename=cp_filename
    )

    # Tune controller
    controller_params['LoggingLevel'] = 2
    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    # Write parameter input file
    sim_dir = os.path.join(EXAMPLE_OUT_DIR,'17_ZeroMQ')
    os.makedirs(sim_dir,exist_ok=True)
    param_filename = os.path.join(sim_dir, 'DISCON_zmq.IN')
    write_DISCON(
        turbine, controller,
        param_file=param_filename,
        txt_filename=cp_filename
    )


    # Load controller library
    controller_int = ROSCO_ci.ControllerInterface(
        discon_lib_path, 
        param_filename=param_filename, 
        sim_name=os.path.join(sim_dir,'sim-zmq')
        )

    # Load the simulator
    sim = ROSCO_sim.Sim(turbine, controller_int)

    # Define a wind speed history
    dt = 0.025
    tlen = 100      # length of time to simulate (s)
    ws0 = 7         # initial wind speed (m/s)
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    # add steps at every 100s
    for i in range(len(t)):
        ws[i] = ws[i] + t[i]//100

    # Define wind directions as zeros  
    wd = np.zeros_like(t)

    # Run simulator and plot results
    sim.sim_ws_wd_series(t, ws, wd, rotor_rpm_init=4, make_plots=True)

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(EXAMPLE_OUT_DIR, '17_NREL5MW_ZMQ.png'))

    # Check that info is passed to ROSCO
    op = output_processing.output_processing()
    local_vars = op.load_fast_out([os.path.join(sim_dir,'sim-zmq.RO.dbg2')], tmin=0)
    fig, axs = plt.subplots(2,1)
    axs[0].plot(local_vars[0]['Time'],local_vars[0]['ZMQ_YawOffset'])
    axs[1].plot(local_vars[0]['Time'],local_vars[0]['ZMQ_PitOffset'])

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(EXAMPLE_OUT_DIR, '17_NREL5MW_ZMQ_Setpoints.png'))

    # Spot check input at time = 30 sec.
    ind_30 = local_vars[0]['Time'] == TIME_CHECK
    np.testing.assert_almost_equal(local_vars[0]['ZMQ_YawOffset'][ind_30], DESIRED_YAW_OFFSET)
    np.testing.assert_almost_equal(local_vars[0]['ZMQ_PitOffset'][ind_30], DESIRED_PITCH_OFFSET, decimal=3)

if __name__ == "__main__":
    main()
