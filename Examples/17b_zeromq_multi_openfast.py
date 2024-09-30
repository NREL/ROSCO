"""
17b_zeromq_multi_openfast
-------------------------
Run multiple openfast simulations and execute communication with ZeroMQ.
"""

import os
import numpy as np
import multiprocessing as mp
import matplotlib.pyplot as plt
from rosco.toolbox.control_interface import wfc_zmq_server
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.fast_io import output_processing
    
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
EXAMPLE_OUT_DIR = os.path.join(THIS_DIR, "examples_out")
os.makedirs(EXAMPLE_OUT_DIR, exist_ok=True)

TIME_CHECK = 20
DESIRED_YAW_OFFSET = [-10, 10]

def main():
    

    # Start wind farm control server and two openfast simulation
    # as separate processes
    logfile = os.path.join(EXAMPLE_OUT_DIR,os.path.splitext(os.path.basename(__file__))[0]+'.log')
    p0 = mp.Process(target=run_zmq,args=(logfile,))
    p1 = mp.Process(target=sim_openfast_1)
    p2 = mp.Process(target=sim_openfast_2)

    p0.start()
    p1.start()
    p2.start()

    p0.join()
    p1.join()
    p2.join()

    ## Run tests
    # Check that info is passed to ROSCO for first simulation
    op1 = output_processing.output_processing()
    debug_file1 = os.path.join(
        EXAMPLE_OUT_DIR,
        "17b_zeromq_OF1",
        "NREL5MW",
        "power_curve",
        "base",
        "NREL5MW_0.RO.dbg2",
    )
    local_vars1 = op1.load_fast_out(debug_file1, tmin=0)

    # Check that info is passed to ROSCO for first simulation
    op2 = output_processing.output_processing()
    debug_file2 = os.path.join(
        EXAMPLE_OUT_DIR,
        "17b_zeromq_OF2",
        "NREL5MW",
        "power_curve",
        "base",
        "NREL5MW_0.RO.dbg2",
    )
    local_vars2 = op2.load_fast_out(debug_file2, tmin=0)

    # Generate plots
    _, axs = plt.subplots(2, 1)
    axs[0].plot(local_vars1[0]["Time"], local_vars1[0]["ZMQ_YawOffset"])
    axs[1].plot(local_vars2[0]["Time"], local_vars2[0]["ZMQ_YawOffset"])

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(EXAMPLE_OUT_DIR, "17b_NREL5MW_ZMQ_Setpoints.png"))

    # Spot check input at time = 30 sec.
    ind1_30 = local_vars1[0]["Time"] == TIME_CHECK
    ind2_30 = local_vars2[0]["Time"] == TIME_CHECK

    np.testing.assert_almost_equal(
        local_vars1[0]["ZMQ_YawOffset"][ind1_30], DESIRED_YAW_OFFSET[0]
    )
    np.testing.assert_almost_equal(
        local_vars2[0]["ZMQ_YawOffset"][ind2_30], DESIRED_YAW_OFFSET[1]
    )


def run_zmq(logfile=None):
    """Start the ZeroMQ server for wind farm control"""

    # Start the server at the following address
    network_address = "tcp://*:5555"
    server = wfc_zmq_server(network_address, timeout=60.0, verbose=False, logfile = logfile)

    # Provide the wind farm control algorithm as the wfc_controller method of the server
    server.wfc_controller = wfc_controller

    # Run the server to receive measurements and send setpoints
    server.runserver()


def wfc_controller(id, current_time, measurements):
    """
    Users needs to define this function to implement wind farm controller.
    The user defined function should take as argument the turbine id, the
    current time and current measurements and return the setpoints
    for the particular turbine for the current time. It should ouput the
    setpoints as a dictionary whose keys should be as defined in
    wfc_zmq_server.wfc_interface. The wfc_controller method of the wfc_zmq_server
    should be overwriten with this fuction, otherwise, an exception is raised and
    the simulation stops.
    """
    if current_time <= 10.0:
        YawOffset = 0.0
        col_pitch_command = 0.0
    else:
        col_pitch_command = np.deg2rad(2) * np.sin(0.1 * current_time) + np.deg2rad(2) # Implement dynamic induction control
        if id == 1:
            YawOffset = DESIRED_YAW_OFFSET[0]
        else:
            YawOffset = DESIRED_YAW_OFFSET[1]

        
    setpoints = {}
    setpoints["ZMQ_YawOffset"] = YawOffset
    setpoints['ZMQ_PitOffset(1)'] = col_pitch_command
    setpoints['ZMQ_PitOffset(2)'] = col_pitch_command
    setpoints['ZMQ_PitOffset(3)'] = col_pitch_command
    return setpoints


def sim_openfast_1():
    """Run the first OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 25,
    }
    run_dir = os.path.join(EXAMPLE_OUT_DIR, "17b_zeromq_OF1")
    r.controller_params = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"] = {}
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 1
    r.save_dir = run_dir
    r.run_FAST()


def sim_openfast_2():
    """Run the second OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 25,
    }
    run_dir = os.path.join(EXAMPLE_OUT_DIR, "17b_zeromq_OF2")
    r.save_dir = run_dir
    r.controller_params = {}
    r.controller_params["DISCON"] = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 2
    r.run_FAST()


if __name__ == "__main__":
    main()
