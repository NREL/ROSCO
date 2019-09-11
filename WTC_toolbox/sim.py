# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

# Try to call the DLL
from ctypes import byref, cdll, c_int, POINTER, c_float, c_char_p, c_double, create_string_buffer, c_int32
import numpy as np
from numpy.ctypeslib import ndpointer

# PARAMETERS
DT = 0.1
num_blade = 3

# discon = cdll.LoadLibrary('./DISCON_glin64.so')

discon = cdll.LoadLibrary('/Users/pfleming/Desktop/git_tools/floating/DRC_Fortran/DISCON//DISCON_glin64.so')
num_item = 500
# avrSWAP = (c_float * num_item)()

# AVR SWAP FILLING
avrSWAP = np.zeros(200)
avrSWAP[2] = DT
avrSWAP[60] = num_blade


# Put some values in
avrSWAP[48] = 9
avrSWAP[49] = 1000
avrSWAP[50] = 1000

aviFAIL = c_int32() # 1
accINFILE ='DISCON.In'.encode('utf-8')
avcOUTNAME = create_string_buffer(1000) # 'DEMO'.encode('utf-8')
avcMSG = create_string_buffer(1000)
print(avrSWAP)

discon.DISCON.argtypes = [ndpointer(c_double, flags="C_CONTIGUOUS"), POINTER(c_int32), c_char_p, c_char_p, c_char_p] # (all defined by ctypes)
discon.DISCON(avrSWAP, byref(aviFAIL), accINFILE, avcOUTNAME, avcMSG)


#discon.DISCON.argtypes = [POINTER(c_float), c_int, c_char_p, c_char_p, c_char_p] # (all defined by ctypes)
# discon.DISCON(byref(avrSWAP), aviFAIL, accINFILE, avcOUTNAME, avcMSG)