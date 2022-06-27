'''
----------- Example_17 --------------
Run ROSCO using the ROSCO toolbox control interface and execute communication with ZeroMQ
-------------------------------------

A demonstrator for ZeroMQ communication. Instead of using ROSCO with with control interface, 
one could call ROSCO from OpenFAST, and communicate with ZeroMQ through that.
'''


import platform
import os
import matplotlib.pyplot as plt
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
from ROSCO_toolbox.utilities import write_DISCON
from ROSCO_toolbox import control_interface as ROSCO_ci
from ROSCO_toolbox.control_interface import turbine_zmq_server
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import controller as ROSCO_controller
import numpy as np
import multiprocessing as mp

def run_zmq():
    connect_zmq = True
    s = turbine_zmq_server(network_address="tcp://*:5555", timeout=10.0, verbose=True)
    while connect_zmq:
        #  Get latest measurements from ROSCO
        measurements = s.get_measurements()

        # Decide new control input based on measurements
        current_time = measurements['Time']
        if current_time <= 10.0:
            yaw_setpoint = 0.0
        else:
            yaw_setpoint = 20.0

            # Send new setpoints back to ROSCO
        s.send_setpoints(nacelleHeading=yaw_setpoint)

        if measurements['iStatus'] == -1:
            connect_zmq = False
            s._disconnect()


def sim_rosco():
    # Load yaml file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir = os.path.join(this_dir, '../Tune_Cases')
    parameter_filename = os.path.join(tune_dir, 'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']

    # Enable ZeroMQ & yaw control
    controller_params['Y_ControlMode'] = 1
    controller_params['ZMQ_Mode'] = 1

    # Specify controller dynamic library path and name
    this_dir = os.path.dirname(os.path.abspath(__file__))
    example_out_dir = os.path.join(this_dir, 'examples_out')
    if not os.path.isdir(example_out_dir):
        os.makedirs(example_out_dir)

    if platform.system() == 'Windows':
        lib_name = os.path.join(this_dir, '../ROSCO/build/libdiscon.dll')
    elif platform.system() == 'Darwin':
        lib_name = os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib')
    else:
        lib_name = os.path.join(this_dir, '../ROSCO/build/libdiscon.so')

    # # Load turbine model from saved pickle
    turbine = ROSCO_turbine.Turbine
    turbine = turbine.load(os.path.join(example_out_dir, '01_NREL5MW_saved.p'))

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(
        tune_dir, path_params['FAST_directory'], path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir, path_params['FAST_directory']),
        dev_branch=True,
        rot_source='txt', txt_filename=cp_filename
    )

    # Tune controller
    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    # Write parameter input file
    param_filename = os.path.join(this_dir, 'DISCON_zmq.IN')
    write_DISCON(
        turbine, controller,
        param_file=param_filename,
        txt_filename=cp_filename
    )


    # Load controller library
    controller_int = ROSCO_ci.ControllerInterface(lib_name, param_filename=param_filename, sim_name='sim-zmq')

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
        plt.savefig(os.path.join(example_out_dir, '16_NREL5MW_zmqYaw.png'))


if __name__ == "__main__":
    p1 = mp.Process(target=run_zmq)
    p1.start()
    p2 = mp.Process(target=sim_rosco)
    p2.start()
    p1.join()
    p2.join()
