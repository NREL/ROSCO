import os
import multiprocessing as mp
from ROSCO_toolbox.control_interface import wfc_zmq_server
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO


this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir, "examples_out")
os.makedirs(example_out_dir, exist_ok=True)


def run_zmq():
    """Start the ZeroMQ server for wind farm control"""

    # Start the server at the following address
    network_address = "tcp://*:5555"
    server = wfc_zmq_server(network_address, timeout=60.0, verbose=True)

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
    else:
        if id == 1:
            YawOffset = -10.0
        else:
            YawOffset = 10
    setpoints = {}
    setpoints["ZMQ_YawOffset"] = YawOffset
    return setpoints


def sim_openfast_1():
    """Run the first OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 100,
    }
    run_dir = os.path.join(example_out_dir, "17b_zeromq_OF1")
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
        "TMax": 100,
    }
    run_dir = os.path.join(example_out_dir, "17b_zeromq_OF2")
    r.save_dir = run_dir
    r.controller_params = {}
    r.controller_params["DISCON"] = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 2
    r.run_FAST()


if __name__ == "__main__":
    # Start wind farm control server and two openfast simulation
    # as separate processes
    p0 = mp.Process(target=run_zmq)
    p1 = mp.Process(target=sim_openfast_1)
    p2 = mp.Process(target=sim_openfast_2)

    p0.start()
    p1.start()
    p2.start()

    p0.join()
    p1.join()
    p2.join()
