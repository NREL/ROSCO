"""
17c_zeromq_ff
-------------
Run FAST.Farm simulation and communication with ZeroMQ.
"""

import os
import numpy as np
import multiprocessing as mp
from rosco.toolbox.control_interface import wfc_zmq_server


this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir, "examples_out")
os.makedirs(example_out_dir, exist_ok=True)
DESIRED_YAW_OFFSET = [-10, 10]


def run_zmq(logfile=None):
    """Start the ZeroMQ server for wind farm control"""

    # Start the server at the following address
    network_address = "tcp://*:5555"
    server = wfc_zmq_server(network_address, timeout=60.0, verbose=True, logfile=logfile)

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


def run_FF():
    fstf_file = ""
    if not fstf_file:
        raise Exception("FAST.Farm input file must be provided in the variable 'fstf_file'")
    FF_cmd = "FAST.Farm " + fstf_file
    os.system(FF_cmd)


if __name__ == "__main__":
    logfile = os.path.join( example_out_dir, os.path.splitext(os.path.basename(__file__))[0] + ".log")
    p0 = mp.Process(target=run_zmq, args=(logfile,))
    p1 = mp.Process(target=run_FF)

    p0.start()
    p1.start()

    p0.join()
    p1.join()
