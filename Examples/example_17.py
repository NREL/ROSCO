import platform
import os
import matplotlib.pyplot as plt
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
from ROSCO_toolbox.utilities import write_DISCON
from ROSCO_toolbox import control_interface as ROSCO_ci
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import controller as ROSCO_controller
import numpy as np
import zmq
import multiprocessing as mp

class farm_zmq_server():
    def __init__(self, network_addresses=["tcp://*:5555", "tcp://*:5556"],
                 identifiers=None, timeout=600.0, verbose=False):
        """Python implementation for communicating with multiple instances
        of the ROSCO ZeroMQ interface. This is useful for SOWFA and FAST.Farm
        simulations in which multiple turbines are running in real time.
        Args:
            network_addresses (str, optional): List with the network addresses
            used to communicate with the desired instances of ROSCO.
            identifiers (iteratible, optional): List of strings denoting the
            turbine identification string, e.g., ["WTG-01", "WTG-02"].
            If left unspecified, will simple name the turbines "0" to
            nturbs - 1.
            timeout (float, optional): Seconds to wait for a message from
            the ZeroMQ server before timing out. Defaults to 600.0.
            verbose (bool, optional): Print to console. Defaults to False.
        """
        self.network_addresses = network_addresses
        self.verbose = verbose
        self.nturbs = len(self.network_addresses)

        if identifiers is None:
            identifiers = ["%d" % i for i in range(self.nturbs)]

        # Initialize ZeroMQ servers
        self.zmq_servers = [None for _ in range(self.nturbs)]
        for ti, address in enumerate(self.network_addresses):
            self.zmq_servers[ti] = turbine_zmq_server(
                network_address=address,
                identifier=identifiers[ti],
                timeout=timeout,
                verbose=verbose)

    def get_measurements(self):
        measurements = [None for _ in range(self.nturbs)]
        for ti in range(self.nturbs):
            measurements[ti] = self.zmq_servers[ti].get_measurements()
        return measurements

    def send_setpoints(self, genTorques=None, nacelleHeadings=None,
                       bladePitchAngles=None):

        # Default choices if unspecified
        if genTorques is None:
            genTorques = [0.0] * self.nturbs
        if nacelleHeadings is None:
            nacelleHeadings = [0.0] * self.nturbs
        if bladePitchAngles is None:
            bladePitchAngles = [[0.0, 0.0, 0.0]] * self.nturbs

        # Send setpoints
        for ti in range(self.nturbs):
            self.zmq_servers[ti].send_setpoints(
                genTorque=genTorques[ti],
                nacelleHeading=nacelleHeadings[ti],
                bladePitch=bladePitchAngles[ti]
            )

class turbine_zmq_server():
    def __init__(self, network_address="tcp://*:5555", identifier="0",
                 timeout=600.0, verbose=False):
        """Python implementation of the ZeroMQ server side for the ROSCO
        ZeroMQ wind farm control interface. This class makes it easy for
        users to receive measurements from ROSCO and then send back control
        setpoints (generator torque, nacelle heading and/or blade pitch
        angles).
        Args:
            network_address (str, optional): The network address to
            communicate over with the desired instance of ROSCO. Note that,
            if running a wind farm simulation in SOWFA or FAST.Farm, there
            are multiple instances of ROSCO and each of these instances
            needs to communicate over a unique port. Also, for each of those
            instances, you will need an instance of zmq_server. This variable
            Defaults to "tcp://*:5555".
            identifier (str, optional): Turbine identifier. Defaults to "0".
            timeout (float, optional): Seconds to wait for a message from
            the ZeroMQ server before timing out. Defaults to 600.0.
            verbose (bool, optional): Print to console. Defaults to False.
        """
        self.network_address = network_address
        self.identifier = identifier
        self.timeout = timeout
        self.verbose = verbose
        self._connect()

    def _connect(self):
        address = self.network_address

        # Connect socket
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(address)

        if self.verbose:
            print("[%s] Successfully established connection with %s" % (self.identifier, address))

    def _disconnect(self):
        self.socket.close()
        context = zmq.Context()
        context.term()

    def get_measurements(self):
        if self.verbose:
            print("[%s] Waiting to receive measurements from ROSCO..." % (self.identifier))

        # Initialize a poller for timeouts
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)
        timeout_ms = int(self.timeout * 1000)
        if poller.poll(timeout_ms):
            # Receive measurements over network protocol
            message_in = self.socket.recv_string()
        else:
            raise IOError("[%s] Connection to '%s' timed out."
                          % (self.identifier, self.network_address))

        # Convert to individual strings and then to floats
        measurements = message_in
        # measurements = bytes.decode(message_in)
        measurements = measurements.replace('\x00', '').split(',')
        measurements = [float(m) for m in measurements]

        # Convert to a measurement dict
        measurements = dict({
            'iStatus': measurements[0],
            'Time': measurements[1],
            'VS_MechGenPwr':  measurements[2],
            'VS_GenPwr': measurements[3],
            'GenSpeed': measurements[4],
            'RotSpeed': measurements[5],
            'GenTqMeas': measurements[6],
            'NacelleHeading': measurements[7],
            'NacelleVane': measurements[8],
            'HorWindV': measurements[9],
            'rootMOOP1': measurements[10],
            'rootMOOP2': measurements[11],
            'rootMOOP3': measurements[12],
            'FA_Acc': measurements[13],
            'NacIMU_FA_Acc': measurements[14],
            'Azimuth': measurements[15],
        })

        if self.verbose:
            print('[%s] Measurements received:' % self.identifier, measurements)

        return measurements

    def send_setpoints(self, genTorque=0.0, nacelleHeading=0.0,
                       bladePitch=[0.0, 0.0, 0.0]):
        # Create a message with setpoints to send to ROSCO
        message_out = b"%016.5f, %016.5f, %016.5f, %016.5f, %016.5f" % (
            genTorque, nacelleHeading, bladePitch[0], bladePitch[1],
            bladePitch[2])

        #  Send reply back to client
        if self.verbose:
            print("[%s] Sending setpoint string to ROSCO: %s." % (self.identifier, message_out))

        # Send control setpoints over network protocol
        self.socket.send(message_out)

        if self.verbose:
            print("[%s] Setpoints sent successfully." % self.identifier)



def run_zmq():
    s = turbine_zmq_server(network_address="tcp://*:5555", timeout=10.0, verbose=True)
    while True:
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

    s.disconnect()


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
        lib_name = os.path.join(this_dir, '../ROSCO/build-zmq/libdiscon.dylib')
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
    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='sim1')

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

    # # Load controller library again to see if we deallocated properly
    # controller_int = ROSCO_ci.ControllerInterface(
    #     lib_name, param_filename=param_filename, sim_name='sim_2')

    # # Run simulator again and plot results
    # sim_2 = ROSCO_sim.Sim(turbine, controller_int)
    # sim_2.sim_ws_series(t, ws, rotor_rpm_init=4)

    # # Check if simulations are equal
    # np.testing.assert_almost_equal(sim_1.gen_speed, sim_2.gen_speed)

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir, '16_NREL5MW_zmqYaw.png'))


if __name__ == "__main__":
    # sim_rosco()

    p1 = mp.Process(target=run_zmq)
    p1.start()
    p2 = mp.Process(target=sim_rosco)
    p2.start()
    p1.join()
    p2.join()
