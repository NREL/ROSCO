import platform
import os
import matplotlib.pyplot as plt
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
from ROSCO_toolbox.utilities import write_DISCON
from ROSCO_toolbox import control_interface as ROSCO_ci
from ROSCO_toolbox.control_interface import wfc_zmq_server
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
import numpy as np
import multiprocessing as mp
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO

this_dir            = os.path.dirname(os.path.abspath(__file__))
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

def run_zmq():
    connect_zmq = True
    s = wfc_zmq_server(network_address="tcp://*:5555", timeout=10000.0, verbose=False, n_turbines = 2)
    while connect_zmq:
        #  Get latest measurements from ROSCO
        measurements = s.get_measurements()
        identifier = measurements['ZMQ_ID']

        # somewhere in the server code, we need to save the ID to setpoints
        # otherwise, it's initialized as 0 like everything else
        # do this here for now until the server code moves somewhere else
        
        # I think we need to build out this part of the code, next
        id = int(identifier)

        s.setpoints[id-1]['ZMQ_ID'] = measurements['ZMQ_ID']

        # Maybe measurements needs to be a dict so we don't have this id-1 business
        s.measurements[id-1] = measurements

        # Decide new control input based on measurements
        current_time = measurements['Time']
        if current_time <= 10.0:
            yaw_setpoint = 0.0
        else:
            if identifier >= 1.5:
                yaw_setpoint = -10.0
            else:
                yaw_setpoint = 10.0
        
        # current_time1 = measurements1['Time']
        # if current_time1 <= 10.0:
        #     yaw_setpoint1 = 0.0
        # else:
        #     yaw_setpoint1 = -20.0

            # Send new setpoints back to ROSCO
        s.setpoints[id-1]['ZMQ_YawOffset'] = yaw_setpoint
        s.send_setpoints(id)
        # s1.send_setpoints(nacelleHeading=yaw_setpoint1)

        if measurements['iStatus'] == -1:
            connect_zmq = False
            s._disconnect()
    print('Done with run_zmq')

def sim_openfast_1():
    # fstfile = '/Users/agupta/Projects/Tools/AG_ROSCO/Examples/IEA-3.4-130-RWT/openfast/IEA-3.4-130-RWT.fst'
    # of_exec = '/Users/agupta/Projects/Tools/AG_openfast/build_v3p5p0/glue-codes/openfast/openfast'
    # os.system(of_exec + ' ' + fstfile)
    # pass
    r = run_FAST_ROSCO()
    r.tuning_yaml = 'NREL5MW.yaml'
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [8],
        'TMax': 100,
        }
    run_dir = os.path.join(example_out_dir,'17b_zeromq_OF1')
    r.controller_params = {}
    r.controller_params['LoggingLevel'] = 2
    r.controller_params['DISCON'] = {}
    r.controller_params['DISCON']['ZMQ_Mode'] = 1
    r.controller_params['DISCON']['ZMQ_ID'] = 1
    r.save_dir    = run_dir
    r.run_FAST()



def sim_openfast_2():
    r = run_FAST_ROSCO()
    r.tuning_yaml = 'NREL5MW.yaml'
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [8],
        'TMax': 100,
        }
    run_dir = os.path.join(example_out_dir,'17b_zeromq_OF2')
    r.save_dir    = run_dir
    r.controller_params = {}
    r.controller_params['DISCON'] = {}
    r.controller_params['LoggingLevel'] = 2
    r.controller_params['DISCON']['ZMQ_Mode'] = 1
    r.controller_params['DISCON']['ZMQ_ID'] = 2
    r.run_FAST()


if __name__ == "__main__":
    # sim_rosco()
    # run_zmq()
    # sim_openfast()
    p1 = mp.Process(target=run_zmq)
    p1.start()
    p2 = mp.Process(target=sim_openfast_1)
    p2.start()
    p3 = mp.Process(target=sim_openfast_2)
    p3.start()
    p1.join()
    p2.join()
    p3.join()

    