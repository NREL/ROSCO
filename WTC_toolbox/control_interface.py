# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

from ctypes import byref, cdll, c_int, POINTER, c_float, c_char_p, c_double, create_string_buffer, c_int32
import numpy as np
from numpy.ctypeslib import ndpointer

# Some useful constants
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0

class ConInt():
    """
    Define interface to a given controller
    """

    def __init__(self, lib_name):
        """
        Setup the interface
        """
        self.lib_name = lib_name
        self.param_name = 'DISCON.IN' # this appears to be hard-coded so match here for now

        # Temp fixed parameters
        # PARAMETERS
        self.DT = 0.1
        self.num_blade = 3
        self.char_buffer = 500
        self.avr_size = 500

        # Initialize
        self.pitch = 0
        self.torque = 0

        self.discon = cdll.LoadLibrary(self.lib_name)

        self.avrSWAP = np.zeros(self.avr_size)

        # Define avrSWAP some
        self.avrSWAP[2] = self.DT
        self.avrSWAP[60] = self.num_blade

        # Code this as first casll
        self.avrSWAP[0] = 0

        # Put some values in
        self.avrSWAP[48] = len(self.param_name)
        self.avrSWAP[49] = self.char_buffer
        self.avrSWAP[50] = self.char_buffer

        self.aviFAIL = c_int32() # 1
        self.accINFILE = self.param_name.encode('utf-8')
        self.avcOUTNAME = create_string_buffer(1000) # 'DEMO'.encode('utf-8')
        self.avcMSG = create_string_buffer(1000)

        # self.discon.DISCON.argtypes = [ndpointer(c_double, flags="C_CONTIGUOUS"), POINTER(c_int32), c_char_p, c_char_p, c_char_p] # (all defined by ctypes)
        self.discon.DISCON.argtypes = [POINTER(c_float), POINTER(c_int32), c_char_p, c_char_p, c_char_p] # (all defined by ctypes)

        self.call_discon()


        # First call
        # self.discon.DISCON(self.avrSWAP, byref(self.aviFAIL), self.accINFILE, self.avcOUTNAME, self.avcMSG)
        

        # Code as not first run
        self.avrSWAP[0] = 1


    def call_discon(self):
        
        # Convert AVR swap to the right thing
        c_float_p = POINTER(c_float)
        data = self.avrSWAP.astype(np.float32)
        data_p = data.ctypes.data_as(c_float_p)

        self.discon.DISCON(data_p, byref(self.aviFAIL), self.accINFILE, self.avcOUTNAME, self.avcMSG)

        # Push back to avr swap
        # print(data_p[47])
        # print(self.avrSWAP[47])
        #for i in range(len(self.avrSWAP)):
        #    self.avrSWAP
        #print('len',len(data_p),len(self.avrSWAP))


        self.avrSWAP = data


    def call_controller(self,t,dt,pitch,genspeed,rotspeed,ws):
        
        
        
        # Add states to avr
        self.avrSWAP[1] = t
        self.avrSWAP[2] = dt
        self.avrSWAP[3] = self.pitch
        self.avrSWAP[19] = genspeed
        self.avrSWAP[20] = rotspeed
        self.avrSWAP[26] = ws

        self.call_discon()
        # self.discon.DISCON(self.avrSWAP, byref(self.aviFAIL), self.accINFILE, self.avcOUTNAME, self.avcMSG)

        self.pitch = self.avrSWAP[41]
        self.torque = self.avrSWAP[47]

        return(self.torque,self.pitch)

    def show_control_values(self):
        print('Pitch',self.pitch)
        print('Torque',self.torque)


#discon.DISCON.argtypes = [POINTER(c_float), c_int, c_char_p, c_char_p, c_char_p] # (all defined by ctypes)
# discon.DISCON(byref(avrSWAP), aviFAIL, accINFILE, avcOUTNAME, avcMSG)