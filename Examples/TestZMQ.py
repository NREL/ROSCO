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
    s = turbine_zmq_server(network_address="tcp://*:5555", timeout=10000.0, verbose=False)
    while connect_zmq:
        #  Get latest measurements from ROSCO
        measurements = s.get_measurements()
        identifier = measurements['Identifier']

        # Decide new control input based on measurements
        current_time = measurements['Time']
        if current_time <= 10.0:
            yaw_setpoint = 0.0
        else:
            if identifier >= 1.5:
                yaw_setpoint = -10.0
            else:
                yaw_setpoint = -10.0
        
        # current_time1 = measurements1['Time']
        # if current_time1 <= 10.0:
        #     yaw_setpoint1 = 0.0
        # else:
        #     yaw_setpoint1 = -20.0

            # Send new setpoints back to ROSCO
        s.send_setpoints(nacelleHeading=yaw_setpoint)
        # s1.send_setpoints(nacelleHeading=yaw_setpoint1)

        if measurements['iStatus'] == -1:
            connect_zmq = False
            s._disconnect()
    print('Done with run_zmq')

def sim_openfast():
    fstfile = '/Users/agupta/Projects/Tools/AG_ROSCO/Examples/IEA-3.4-130-RWT/openfast/IEA-3.4-130-RWT.fst'
    of_exec = '/Users/agupta/Projects/Tools/AG_openfast/build_v3p5p0/glue-codes/openfast/openfast'
    os.system(of_exec + ' ' + fstfile)
    pass

def sim_openfast1():
    fstfile = '/Users/agupta/Projects/Tools/AG_ROSCO/Examples/IEA-3.4-130-RWT1/openfast/IEA-3.4-130-RWT.fst'
    of_exec = '/Users/agupta/Projects/Tools/AG_openfast/build_v3p5p0/glue-codes/openfast/openfast'
    os.system(of_exec + ' ' + fstfile)
    pass


if __name__ == "__main__":
    # sim_rosco()
    # run_zmq()
    # sim_openfast()
    p1 = mp.Process(target=run_zmq)
    p1.start()
    # p2 = mp.Process(target=sim_openfast)
    # p2.start()
    p3 = mp.Process(target=sim_openfast1)
    p3.start()
    p1.join()
    # p2.join()
    p3.join()

    