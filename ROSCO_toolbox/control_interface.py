# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

from ctypes import byref, cdll, c_int, POINTER, c_float, c_char_p, c_double, create_string_buffer, c_int32, c_void_p
import numpy as np
from numpy.ctypeslib import ndpointer
import platform, ctypes

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


        # Code this as first casll
        self.avrSWAP[0] = 0

        # Put some values in
        self.avrSWAP[58] = self.char_buffer
        self.avrSWAP[49] = len(self.param_name)
        self.avrSWAP[50] = self.char_buffer
        self.avrSWAP[51] = self.char_buffer

        # Initialize DISCON and related
        self.aviFAIL = c_int32() # 1
        self.accINFILE = self.param_name.encode('utf-8')
        # self.avcOUTNAME = create_string_buffer(1000) # 'DEMO'.encode('utf-8')
        self.avcOUTNAME = (self.sim_name + '.RO.dbg').encode('utf-8')
        self.avcMSG = create_string_buffer(1000)
        self.discon.DISCON.argtypes = [POINTER(c_float), POINTER(c_int32), c_char_p, c_char_p, c_char_p] # (all defined by ctypes)

        # Run DISCON
        self.call_discon()

        # Code as not first run
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


    def call_controller(self,t,dt,pitch,torque,genspeed,geneff,rotspeed,ws,NacIMU_FA_Acc=0):
        '''
        Runs the controller. Passes current turbine state to the controller, and returns control inputs back
        
        Parameters:
        -----------
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
        NacIMU_FA_Acc : float
            nacelle IMU accel. in the nodding dir. , (m/s**2)
            default to 0 (fixed-bottom, simple 1-DOF sim does not include it, but OpenFAST linearizations do)
        '''

        # Add states to avr
        self.avrSWAP[1] = t
        self.avrSWAP[2] = dt
        self.avrSWAP[3] = pitch
        self.avrSWAP[32] = pitch
        self.avrSWAP[33] = pitch
        self.avrSWAP[14] = genspeed*torque*geneff
        self.avrSWAP[22] = torque
        self.avrSWAP[19] = genspeed
        self.avrSWAP[20] = rotspeed
        self.avrSWAP[26] = ws
        self.avrSWAP[82] = NacIMU_FA_Acc

        # call controller
        self.call_discon()

        # return controller states
        self.pitch = self.avrSWAP[41]
        self.torque = self.avrSWAP[46]

        return(self.torque,self.pitch)

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
