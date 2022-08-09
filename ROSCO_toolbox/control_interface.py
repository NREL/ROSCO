# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

from ctypes import byref, cdll, POINTER, c_float, c_char_p, c_double, create_string_buffer, c_int32, c_void_p
import numpy as np
import platform, ctypes
import zmq

# Some useful constants
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0

class ControllerInterface():
    """
    Define interface to a given controller using the avrSWAP array

    Methods:
    --------
    call_discon
    call_controller
    show_control_values

    Parameters:
    -----------
    lib_name : str
                name of compiled dynamic library containing controller, (.dll,.so,.dylib)

    """

    def __init__(self, lib_name, param_filename='DISCON.IN', **kwargs):
        """
        Setup the interface
        """
        self.lib_name = lib_name
        self.param_name = param_filename

        # Set default parameters
        # PARAMETERS
        self.DT = 0.1
        self.num_blade = 3
        self.char_buffer = 500
        self.avr_size = 500
        self.sim_name = 'simDEBUG'

        # Set kwargs, like DT
        for (k, w) in kwargs.items():
            try:
                setattr(self, k, w)
            except:
                pass

        self.init_discon()

    def init_discon(self):

        # Initialize
        self.pitch = 0
        self.torque = 0
        # -- discon
        self.discon = cdll.LoadLibrary(self.lib_name)
        self.avrSWAP = np.zeros(self.avr_size)

        # Define some avrSWAP parameters, NOTE: avrSWAP indices are offset by -1 from Fortran
        self.avrSWAP[2] = self.DT
        self.avrSWAP[60] = self.num_blade
        self.avrSWAP[19] = 1.0 # HARD CODE initial gen speed = 1 rad/s
        self.avrSWAP[20] = 1.0 # HARD CODE initial rot speed = 1 rad/s
        self.avrSWAP[82] = 0 # HARD CODE initial nacIMU = 0
        self.avrSWAP[26] = 10 # HARD CODE initial wind speed = 10 m/s
        
        # Blade pitch initial conditions
        self.avrSWAP[3]     = 0 * np.deg2rad(1)
        self.avrSWAP[32]    = 0 * np.deg2rad(1)
        self.avrSWAP[33]    = 0 * np.deg2rad(1)

        # Torque initial condition
        self.avrSWAP[22]    = 0

        # Code this as first call
        self.avrSWAP[0] = 0

        # Put some values in
        self.avrSWAP[58] = self.char_buffer
        self.avrSWAP[49] = len(self.param_name)
        self.avrSWAP[50] = self.char_buffer
        self.avrSWAP[51] = self.char_buffer

        # Initialize DISCON and related
        self.aviFAIL = c_int32() # 1
        self.accINFILE = self.param_name.encode('utf-8')
        self.avcOUTNAME = (self.sim_name + '.RO.dbg').encode('utf-8')
        self.avcMSG = create_string_buffer(1000)
        self.discon.DISCON.argtypes = [POINTER(c_float), POINTER(c_int32), c_char_p, c_char_p, c_char_p] # (all defined by ctypes)

        # Run DISCON
        self.call_discon()

        # Code as not first run now that DISCON has been initialized
        self.avrSWAP[0] = 1


    def call_discon(self):
        '''
        Call libdiscon.dll (or .so,.dylib,...)
        '''
        # Convert AVR swap to the c pointer
        c_float_p = POINTER(c_float)
        data = self.avrSWAP.astype(np.float32)
        p_data = data.ctypes.data_as(c_float_p)

        # Run DISCON
        self.discon.DISCON(p_data, byref(self.aviFAIL), self.accINFILE, self.avcOUTNAME, self.avcMSG)

        # Push back to avr swap
        self.avrSWAP = data


    def call_controller(self, turbine_state, end=False): 
        '''
        Runs the controller. Passes current turbine state to the controller, and returns control inputs back
        
        Parameters:
        -----------
        turbine_state: dict
            t: float
                time, (s)
            dt: float
                timestep, (s)
            pitch: float
                blade pitch, (rad)
            genspeed: float
                    generator speed, (rad/s)
            geneff: float
                    generator efficiency, (rad/s)
            rotspeed: float
                    rotor speed, (rad/s)
            ws: float
                wind speed, (m/s)
            yaw: float, optional
                nacelle yaw position (from north) (deg)
            yawerr: float, optional
                yaw misalignment, defined as the wind direction minus the yaw
                position (deg)
        '''

        # Add states to avr
        self.avrSWAP[0] = turbine_state['iStatus']
        self.avrSWAP[1] = turbine_state['t']
        self.avrSWAP[2] = turbine_state['dt']
        self.avrSWAP[3] =  turbine_state['bld_pitch']
        self.avrSWAP[32] = turbine_state['bld_pitch']
        self.avrSWAP[33] = turbine_state['bld_pitch']
        self.avrSWAP[14] = turbine_state['gen_speed'] * turbine_state['gen_torque'] * turbine_state['gen_eff']
        self.avrSWAP[22] = turbine_state['gen_torque']
        self.avrSWAP[19] = turbine_state['gen_speed']
        self.avrSWAP[20] = turbine_state['rot_speed']
        self.avrSWAP[23] = turbine_state['Y_MeasErr']
        self.avrSWAP[26] = turbine_state['ws']
        self.avrSWAP[36] = turbine_state['Yaw_fromNorth']
        try:
            self.avrSWAP[82] = turbine_state['NacIMU_FA_Acc']
        except KeyError:
            self.avrSWAP[82] = 0

        # call controller
        self.call_discon()

        # return controller states
        self.pitch = self.avrSWAP[41]
        self.torque = self.avrSWAP[46]
        self.nac_yawrate = self.avrSWAP[47]

        return(self.torque,self.pitch,self.nac_yawrate)

    def show_control_values(self):
        '''
        Show control values - should be obvious
        '''
        print('Pitch',self.pitch)
        print('Torque',self.torque)

    def kill_discon(self):
        '''
        Unload the dylib from memory: https://github.com/bwoodsend/cslug/blob/master/cslug/_stdlib.py
        '''

        print('Shutting down {}'.format(self.lib_name))
        handle = self.discon._handle

        # Start copy here
        OS = platform.system()

        def null_free_dll(*spam):  # pragma: no cover
            pass

        # Try to find a good runtime library which is always available and contains
        # the standard library C functions such as malloc() or printf().
        # XXX: Keep chosen library names in sync with the table in `cslug/stdlib.py`.

        extra_libs = []

        if OS == "Windows":  # pragma: Windows
            _dlclose = ctypes.windll.kernel32.FreeLibrary
            dlclose = lambda handle: 0 if _dlclose(handle) else 1
            # There's some controversy as to whether this DLL is guaranteed to exist.
            # It always has so far but isn't documented. However, MinGW assumes that it
            # is so, should this DLL be removed, then we have much bigger problems than
            # just this line. There is also vcruntime140.dll which isn't a standard part
            # of the OS but is always shipped with Python so we can guarantee its
            # presence. But vcruntime140 contains only a tiny strict-subset of msvcrt.
            stdlib = ctypes.CDLL("msvcrt")

        elif OS == "Darwin":  # pragma: Darwin
            try:
                try:
                    # macOS 11 (Big Sur). Possibly also later macOS 10s.
                    stdlib = ctypes.CDLL("libc.dylib")
                except OSError:  # pragma: no cover
                    stdlib = ctypes.CDLL("libSystem")
            except OSError:  # pragma: no cover
                # Older macOSs. Not only is the name inconsistent but it's
                # not even in PATH.
                _stdlib = "/usr/lib/system/libsystem_c.dylib"
                if os.path.exists(_stdlib):
                    stdlib = ctypes.CDLL(_stdlib)
                else:
                    stdlib = None
            if stdlib is not None:  # pragma: no branch
                dlclose = stdlib.dlclose
            else:  # pragma: no cover
                # I hope this never happens.
                dlclose = null_free_dll

        elif OS == "Linux":  # pragma: Linux
            try:
                stdlib = ctypes.CDLL("")
            except OSError:  # pragma: no cover
                # Either Alpine Linux or Android.
                # Unfortunately, there doesn't seem to be any practical way
                # to tell them apart.
                stdlib = ctypes.CDLL("libc.so")

                # Android, like FreeBSD puts its math functions
                # in a dedicated `libm.so`.
                # The only way to know that this is not Alpine is to check if the math
                # functions are already available in `libc.so`.
                if not hasattr(stdlib, "sin"):
                    extra_libs.append(ctypes.CDLL("libm.so"))
            dlclose = stdlib.dlclose

        # End copy here
        dlclose.argtypes = [c_void_p]
        dlclose(handle)

        del self.discon


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
        '''
        Get measurements from zmq servers
        '''
        measurements = [None for _ in range(self.nturbs)]
        for ti in range(self.nturbs):
            measurements[ti] = self.zmq_servers[ti].get_measurements()
        return measurements

    def send_setpoints(self, genTorques=None, nacelleHeadings=None,
                       bladePitchAngles=None):
        '''
        Send setpoints to DLL via zmq server for farm level controls

        Parameters:
        -----------
        genTorques: List
            List of generator torques of length self.nturbs
        nacelleHeadings: List
            List of nacelle headings of length self.nturbs
        bladePitchAngles: List
            List of blade pitch angles of length self.nturbs
        '''
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
        '''
        Connect to zmq server
        '''
        address = self.network_address

        # Connect socket
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(address)

        if self.verbose:
            print("[%s] Successfully established connection with %s" % (self.identifier, address))

    def _disconnect(self):
        '''
        Disconnect from zmq server
        '''
        self.socket.close()
        context = zmq.Context()
        context.term()

    def get_measurements(self):
        '''
        Receive measurements from ROSCO .dll
        '''
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
        '''
        Send setpoints to ROSCO .dll ffor individual turbine control

        Parameters:
        -----------
        genTorques: float
            Generator torque setpoint
        nacelleHeadings: float
            Nacelle heading setpoint
        bladePitchAngles: List (len=3)
            Blade pitch angle setpoint
        '''
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
