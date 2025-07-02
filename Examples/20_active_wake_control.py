"""
20_active_wake_control
----------------------
Run openfast with ROSCO and active wake control
Set up and run simulation with AWC, check outputs
Active wake control (AWC) with blade pitching is implemented in this example with two approaches as detailed below:
"""
# -----------------------------------------------
# AWC_Mode = 1: Normal mode method:
# -----------------------------------------------
# The normal mode method is an adaptation into the rotating frame of the mathematical framework from the classical theory for stability of axisymmetric jets [1], which offers flexibility in specifying the forcing strategy.

# The inputs to the controller are:
#     Name			Unit		Type		Range		Description
#     AWC_NumModes	-		Integer		[0,inf]		number of forcing modes
#     AWC_n			-		Integer		[-inf,inf]	azimuthal mode number(s) (i.e., the azimuthal mode number relates to the number and direction of the lobes of the wake structure according to the classical spatio-temporal Fourier decomposition of an arbitrary quantity q, sigma{sigma{q*exp(i*n*theta)*exp(i*omega*time)}}. For the case of a non-time-varying flow (i.e., where omega = 0), the azimuthal mode number specifies the number of cycles of blade pitch oscillation per one rotation around the rotor azimuth.)
#     AWC_clockangle 	deg		Float		[0,360]		clocking angle(s) of forcing mode(s)
#     AWC_freq 		Hz		Float		[0,inf]		frequency(s) of forcing mode(s)
#     AWC_amp 		deg		Float		[0,inf]		pitch amplitude(s) of forcing mode(s) (note that AWC_amp specifies the amplitude of each individual mode so that the total amplitude of pitching will be the sum of AWC_amp)

# The latter two inputs may be specified based on the expected inflow while the former three inputs determine the type of active wake control to be used.

# Readers may be familiar with several forcing strategies from literature on active wake control that can be represented as follows:
#     -collective dynamic induction control: 	AWC_NumModes = 1, AWC_n = 0, AWC_clockangle = 0
#     -helix clockwise [2]:			        AWC_NumModes = 1, AWC_n = 1, AWC_clockangle = 0
#     -helix counter-clockwise [2]:		    AWC_NumModes = 1, AWC_n = -1, AWC_clockangle = 0
#     -up-and-down:				            AWC_NumModes = 2, AWC_n = -1 1, AWC_clockangle = 0 0
#     -side-to-side:				            AWC_NumModes = 2, AWC_n = -1 1, AWC_clockangle = 90 90
#     -other:					                Higher-order modes or different combinations of the above can also be specified
    
#     These strategies are implemented using the following calculation methodology:
#         For each blade, we compute the total phase angle of blade pitch excursion according to:
#             AWC_angle(t) = 2*Pi*AWC_freq * t - AWC_n * (psi(t) + phi + AWC_clockangle*PI/180)							                        (eq 1)
#             where 	t is time
#                 phi(t) is the angular offset of the given blade in the rotor plane relative to blade 1
#                 psi is the angle of blade 1 in the rotor plane from top-dead center
                
#         Next, the phase angle is converted into the complex pitch amplitude:
#             AWC_complexangle(t) = AWC_amp*PI/180 * EXP(i * AWC_angle(t))										                                (eq 2)
#             where 	i is the square root of -1 
            
#         Note that if AWC_NumModes>1, then eq 1 and 2 are computed for each additional mode, and AWC_complexangle becomes a summation over all modes for each blade.
        
#         Finally, the real pitch amplitude, theta(t), to be passed to the next step of the controller is calculated:
#             theta(t) = theta_0(t) + REAL(AWC_complexangle(t))											                                        (eq 3)
#             where 	theta_0(t) is the controller's nominal pitch command
        
#     Rearranging for ease of viewing:		
#         Inserting eq 1 into eq 2, and then putting that result into eq 3 gives:
#             theta(t) = theta_0(t) + REAL(AWC_amp*PI/180 * EXP(i * (2*Pi*AWC_freq * t - AWC_n * (psi(t) + phi + AWC_clockangle*PI/180))))		(eq 4)
        
#         Applying Euler's formula and carrying out the REAL operator:
#             theta(t) = theta_0(t) + AWC_amp*PI/180 * cos(2*Pi*AWC_freq * t - AWC_n * (psi(t) + phi + AWC_clockangle*PI/180))			        (eq 5)
            
#     As an example, we can set parameters to produce the counter-clockwise helix pattern from [2] using AWC_NumModes = 1, AWC_n = -1, and AWC_clockangle = 0:
#         For blade 1, eq 5 becomes:
#             theta(t) = theta_0(t) + AWC_amp*PI/180 * cos(2*Pi*AWC_freq * t + psi(t))								                            (eq 6)	
            
# Note that the inverse multi-blade coordinate (MBC) transformation can also be used to obtain the same result as eq 6.
#     Beginning with Eq. 3 from [2], we have 
    
#         /            \			         /               \
#         | theta_1(t) |                	 | theta_0(t)    |
#         | theta_2(t) | = T^-1(psi(t)) *  | theta_tilt(t) |												                                        (eq 7)
#         | theta_3(t) |               	 | theta_yaw(t)  |
#         \            /               	 \               /
        
#         where
        
#                         /                     	     	\
#                         | 1 cos(psi_1(t)) sin(psi_1(t)) |
#         T^-1(psi(t)) =  | 1 cos(psi_2(t)) sin(psi_2(t)) |
#                         | 1 cos(psi_3(t)) sin(psi_3(t)) |
#                         \			                    /
                        
#     Multiplying the first row of the top matrix (and dropping the subscript of blade 1) yields:
#         theta(t) = theta_0(t) + theta_tilt(t)*cos(psi(t)) + theta_yaw(t)*sin(psi(t))									                        (eq 8)
        
#     Setting theta_tilt(t) = AWC_amp*PI/180 * cos(2*Pi*AWC_freq * t) and theta_yaw(t) = -AWC_amp*PI/180 * sin(2*Pi*AWC_freq * t) gives:
#         theta(t) = theta_0(t) + (AWC_amp*PI/180 * cos(2*Pi*AWC_freq * t))*cos(psi(t)) - (AWC_amp*PI/180 * sin(2*Pi*AWC_freq * t))*sin(psi(t))	(eq 9)
        
#     Applying a Ptolemy identity gives:
#         theta(t) = theta_0(t) + AWC_amp*PI/180 * cos(2*Pi*AWC_freq * t + psi(t))									                            (eq 10)	
#         which is equivlanet to eq 6 above.

# -----------------------------------------------
# AWC_Mode = 2: Coleman transform method:
# -----------------------------------------------
# A second method is the Coleman transform method.

# The inputs to the controller are:
#     Name			Unit		Type			Range		Description
#     AWC_NumModes	-		Integer				[1,2]		number of modes for tilt and yaw (1: identical settings for tilt and yaw pitch angles, 2: seperate settings for tilt and yaw moments)
#     AWC_harmonic    -		Integer				[0,inf]		harmonic(s) to apply in the inverse Coleman transform (size = AWC_NumModes. 0: collective pitch AWC, 1: 1P IPC-AWC, 2: 2P IPC-AWC, etc.)
#     AWC_clockangle 	deg		Array of Floats		[-360,360]	clocking angle(s) of tilt and yaw pitch angles (size = AWC_NumModes. If size = 1, yaw clockangle = 2*clockangle)
#     AWC_freq 		Hz		Array of Floats		[0,inf]		frequency(s) of the tilt and yaw ptich angles, respectively (size = AWC_NumModes. If size = 1, both frequencies are assumed identical)
#     AWC_amp 		deg		Array of Floats		[0,inf]		pitch amplitude(s) of tilt and yaw pitch angles (size = AWC_NumModes. If size = 1, both amplitudes are assumed identical)
    
# Using the inputs mentioned above, the user is able to specify any desired combination of sinusoidal tilt and yaw modes to be tracked by the turbine.
# When a single mode is defined in the inputs, the prescribed tilt and yaw angles are assumed to be identical, except for the phase. The phase difference
# between the tilt and yaw angles is taken from the input AWC_clockangle.

# Readers may be familiar with several forcing strategies from literature on active wake control that can be represented as follows:
#     -collective dynamic induction control: 	AWC_NumModes = 1, AWC_harmonic = 0
#     -helix clockwise [2]:					AWC_NumModes = 1, AWC_harmonic = 1, AWC_clockangle = -90	OR	AWC_NumModes = 2, AWC_n = [1 1], AWC_clockangle = [0 -90]
#     -helix counter-clockwise [2]:			AWC_NumModes = 1, AWC_harmonic = 1, AWC_clockangle = 90	    OR	AWC_NumModes = 2, AWC_n = [1 1], AWC_clockangle = [0 90]
#     -up-and-down:							AWC_NumModes = 2, AWC_harmonic = [1 1], AWC_amp = [# 0] (where "#" represents the desired amplitude)
#     -side-to-side:							AWC_NumModes = 2, AWC_harmonic = [1 1], AWC_amp = [0 #] (where "#" represents the desired amplitude)
#     -other:									different combinations of the above can also be specified
    
#     These strategies are implemented using the following calculation methodology:
#         The inputs described above enable the user to specify a desired sinusoidal signal for either the collective pitch (AWC_n = 0) or tilt and yaw pitch 
#         angles (AWC_n = 1). These AWC pitch angles are defined as:
#             AWC_angle(t) = AWC_amp * sin(2*pi*AWC_freq*t + AWC_clockangle)									                                    (eq 1)
            
#         In case of collective pitch AWC, this signal is directly superimposed on the regular pitch control signal. 
        
#         In case of IPC-based AWC, the reference tilt and yaw pitch angles theta are transformed to the rotating frame (i.e., pitch angles theta_k(t) for all 
#         individual blades) using the inverse MBC transformation:
        
#             /            \			 		 /               \
#             | theta_1(t) |                	 | theta_0(t)    |
#             | theta_2(t) | = T^-1(psi(t)) *  | theta_tilt(t) |												                                    (eq 2)
#             | theta_3(t) |               	 | theta_yaw(t)  |
#             \            /               	 \               /
        
#         where
        
#             theta_tilt(t) = AWC_amp(1) * sin(2*pi*AWC_freq(1)*t + AWC_clockangle(1))						                                    (eq 3)
#             theta_yaw(t)  = AWC_amp(2) * sin(2*pi*AWC_freq(2)*t + AWC_clockangle(2))						                                    (eq 4)
            
#         and
#                              /                     	     		  \
#                              | 1 	cos(psi_1(t)) 	sin(psi_1(t)) |
#             T^-1(psi(t))  =  | 1 	cos(psi_2(t)) 	sin(psi_2(t)) |											                                    (eq 5)
#                              | 1 	cos(psi_3(t)) 	sin(psi_3(t)) |
#                              \			        				  /
                             
#         with psi_k(t) the azimuthal position of blade k at time instant t. Note that if AWC_NumModes = 1, it is assumed that:
#             AWC_amp(2)  	  = AWC_amp(1)
#             AWC_freq(2) 	  = AWC_freq(1)
#             AWC_clockangle(2) = 2*AWC_clockangle(1)
            
#         For more information on this control strategy, the user is referred to [2].

# -----------------------------------------------
# AWC_Mode = 3: Closed-loop PI control:
# -----------------------------------------------
# This method uses closed-loop PI control to determine the blade pitch based on out-of-plane bending moments.
# Details on this method can be found in Frederik, 2025 [3]. 

# Setting this method up works with much of the same input settings as AWC_Mode = 2. 
# It is strongly recommended to use AWC_NumModes = 2 for IPC-based AWM, and AWC_NumModes = 1 for collective pitch AWM (pulse/DIC).
# AWC_harmonic, AWC_freq, and AWC_clockangle can be used in the same way as described for mode 2.

# Note that the blade pitch and blade moments are inversely correlated. To accomodate this difference and make results with the same AWC_clockangle comparable
# between the open-loop and closed-loop modes, the implemented closed-loop (blade moment) reference has a 180 degree phase offset compared to the open-loop 
# (blade pitch) reference, i.e.:

# Tilt_moment_reference = AWC_amp(1) * sin(2 * pi * AWC_freq(1) * time + AWC_clockangle(1) + pi)
# Yaw_moment_reference  = AWC_amp(2) * sin(2 * pi * AWC_freq(2) * time + AWC_clockangle(2) + pi)

# AWC_amp now describes the amplitude of the blade bending moments, not the pitch angle, and should therefore be chosen much larger (typically in the 1e6-1e7 range).

# Two new inputs are added: AWC_phaseoffset and AWC_CntrGains
# AWC_phaseoffset (scalar) defines the azimuth offset that optimally decouples the tilt and yaw moments, see, e.g., [3], [4].
# AWC_CntrGains (vector size 2) defines the controller gains, Kp and Ki, respectively. As a starting point, these can be chosen
# the same or similar to the controller gains of 1P IPC for load reduction (IPC_KP and IPC_KI).

# This mode has a startup period of 1/AWC_freq(1), to allow for the

# -----------------------------------------------
# AWC_Mode = 4: Closed-loop PR control:
# -----------------------------------------------
# This method uses closed-loop proportional-resonant (PR) control to determine the blade pitch based on out-of-plane bending moments.
# Details on this method can be found in Frederik, 2025 [3]. 
# Details on PR controllers can be found in Zmood, 1999 [5].

# See mode 3 for input guidance. 
# The input parameters can be set in exactly the same way. 
# The second entry of AWC_CntrGains now represents the resonant gain Kr.

# -----------------------------------------------
# AWC_Mode = 5: Closed-loop Strouhal transform control:
# -----------------------------------------------
# This method ONLY WORKS FOR THE HELIX APPROACH.
# It uses an additional transformation on the fixed frame moments to project the loads to the so-called Strouhal frame.
# In the Strouhal frame, the periodic component is removed, leaving a constant moment, the helix moment, which is regulated to the desired
# blade moment amplitude of the helix approach.

# The Strouhal transformation is defined as:

# helix_moment = sin(2*pi*AWC_freq(1)*time + AWC_clockangle(1) ) * (tilt_moment - mean(tilt_moment) ) 
#                        + sin(2*pi*AWC_freq(2)*time + AWC_clockangle(2) ) * (yaw_moment - mean(yaw_moment) )

# Note that although the user can define different AWC_freq for the tilt and yaw moments, this method only works if AWC_freq(1) = AWC_freq(2)
# Furthermore, abs(AWC_clockangle(1) - AWC_clockangle(2)) should always be 90 degrees--a prerequisite for the helix approach.

# In the Strouhal frame, the helix moment is regulated using a single PI controller, and its gains are defined through AWC_CntrGains as
# described for mode 3.
        
# -----------------------------------------------

# General Implementation note: AWC strategies will be compromised if the AWC pitch command attempts to lower the blade pitch below value PC_MinPit as specified 
# in the DISCON file, so PC_MinPit may need to be reduced by the user.

# References:
# [1] - Batchelor, G. K., and A. E. Gill. "Analysis of the stability of axisymmetric jets." Journal of fluid mechanics 14.4 (1962): 529-551.
# [2] - Frederik, Joeri A., et al. "The helix approach: Using dynamic individual pitch control to enhance wake mixing in wind farms." Wind Energy 23.8 (2020): 1739-1751.
# [3] - Frederik, Joeri A., "Closed-loop wind turbine controllers for active wake mixing strategies." Pre-print (2025).
# [4] - Mulders, Sebastiaan P., et al. "Analysis and optimal individual pitch control decoupling by inclusion of an azimuth offset in the multiblade coordinate transformation." 
#           Wind Energy 22.3 (2019): 341-359.
# [5] - Zmood, Daniel Nahum, Donald Grahame Holmes, and Gerwich Bode. "Frequency domain analysis of three phase linear current regulators." Conference Record of the 1999 IEEE 
#           Industry Applications Conference. Thirty-Fourth IAS Annual Meeting (Cat. No. 99CH36370). Vol. 2. IEEE, 1999.


import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.utilities import read_DISCON #, DISCON_dict
#import numpy as np

def main():
    # Choose your implementation method
    AWC_Mode 			= 3 		# 1 for SNL implementation, 2 for Coleman Transformation implementation
                                    # 3 for closed-loop PI, 4 for closed-loop PR
                                    # 5 for closed-loop Strouhal transformed (only works for helix)


    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)
    
    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/NREL2p8.yaml')  # will be dummy and overwritten with SNL DISCON params
    run_dir = os.path.join(example_out_dir,'20_active_wake_control/all_cases')
    os.makedirs(run_dir,exist_ok=True)

    # Read all DISCON inputs
    rosco_vt = read_DISCON(os.path.join(this_dir,'Test_Cases','NREL_2p8_127/NREL-2p8-127_DISCON.IN'))

    # Apply all discon variables as case inputs
    control_base_case = {}
    for discon_input in rosco_vt:
        control_base_case[('DISCON_in',discon_input)] = {'vals': [rosco_vt[discon_input]], 'group': 0}

    # Set up AWC cases defined above
    if AWC_Mode == 1:
        control_base_case[('DISCON_in','AWC_Mode')] = {'vals': [0,1,1,1,1,1], 'group': 2}
        control_base_case[('DISCON_in','AWC_NumModes')] = {'vals': [1,1,1,1,2,2], 'group': 2}
        control_base_case[('DISCON_in','AWC_n')] = {'vals': [[0],[0],[1],[-1],[-1,1], [-1,1]], 'group': 2}
        control_base_case[('DISCON_in','AWC_freq')] = {'vals': [[0],[0.05],[0.05],[0.05],[0.05,0.05], [0.05,0.05]], 'group': 2}
        control_base_case[('DISCON_in','AWC_amp')] = {'vals': [[0],[2.5],[2.5],[2.5],[1.25,1.25], [1.25,1.25]], 'group': 2}
        control_base_case[('DISCON_in','AWC_clockangle')] = {'vals': [[0],[0],[0],[0],[0,0], [90,90]], 'group': 2}
    elif AWC_Mode == 2:
        control_base_case[('DISCON_in','AWC_Mode')] = {'vals': [0,2,2,2,2,2], 'group': 2}
        control_base_case[('DISCON_in','AWC_NumModes')] = {'vals': [1,1,2,2,2,2], 'group': 2}
        control_base_case[('DISCON_in','AWC_harmonic')] = {'vals': [[0],[0],[1,1],[1,1],[1,1],[1,1]], 'group': 2}
        control_base_case[('DISCON_in','AWC_freq')] = {'vals': [[0],[0.05],[0.05,0.05],[0.05,0.05],[0.05,0.05],[0.05,0.05]], 'group': 2}
        control_base_case[('DISCON_in','AWC_amp')] = {'vals': [[0],[2.5],[2.5,2.5],[2.5,2.5],[2.5,0.0],[0.0,2.5]], 'group': 2}
        control_base_case[('DISCON_in','AWC_clockangle')] = {'vals': [[0],[0],[0,-90],[0,90],[0,0],[180,180]], 'group': 2}
    elif (AWC_Mode == 3) or (AWC_Mode == 4):
        control_base_case[('DISCON_in','AWC_Mode')] = {'vals': [0,AWC_Mode,AWC_Mode,AWC_Mode,AWC_Mode,AWC_Mode], 'group': 2}
        control_base_case[('DISCON_in','AWC_NumModes')] = {'vals': [1,2,2,2,2,2], 'group': 2}
        control_base_case[('DISCON_in','AWC_harmonic')] = {'vals': [[0],[0],[1,1],[1,1],[1,1],[1,1]], 'group': 2}
        control_base_case[('DISCON_in','AWC_freq')] = {'vals': [[0],[0.05],[0.05,0.05],[0.05,0.05],[0.05,0.05],[0.05,0.05]], 'group': 2}
        control_base_case[('DISCON_in','AWC_amp')] = {'vals': [[0],[10e6],[5e6,5e6],[5e6,5e6],[5e6,0.0],[0.0,5e6]], 'group': 2}
        control_base_case[('DISCON_in','AWC_clockangle')] = {'vals': [[0],[0],[0,-90],[0,90],[0,0],[180,180]], 'group': 2}
        control_base_case[('DISCON_in','AWC_phaseoffset')] = {'vals': [20,20,20,20,20,20], 'group': 2}
        control_base_case[('DISCON_in','AWC_CntrGains')] = {'vals': [[1e-10,2e-9],[1e-10,2e-9],[1e-10,2e-9],[1e-10,2e-9],[1e-10,2e-9],[1e-10,2e-9]], 'group': 2}
    elif AWC_Mode == 5:
        control_base_case[('DISCON_in','AWC_Mode')] = {'vals': [0,5,5], 'group': 2}
        control_base_case[('DISCON_in','AWC_NumModes')] = {'vals': [1,2,2], 'group': 2}
        control_base_case[('DISCON_in','AWC_harmonic')] = {'vals': [[0],[1,1],[1,1]], 'group': 2}
        control_base_case[('DISCON_in','AWC_freq')] = {'vals': [[0],[0.05,0.05],[0.05,0.05]], 'group': 2}
        control_base_case[('DISCON_in','AWC_amp')] = {'vals': [[0],[5e6,5e6],[5e6,5e6]], 'group': 2}
        control_base_case[('DISCON_in','AWC_clockangle')] = {'vals': [[0],[0,-90],[0,90]], 'group': 2}
        control_base_case[('DISCON_in','AWC_phaseoffset')] = {'vals': [20,20,20], 'group': 2}
        control_base_case[('DISCON_in','AWC_CntrGains')] = {'vals': [[1e-10,2e-9],[1e-10,2e-9],[1e-10,2e-9]], 'group': 2}

    
    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve  # single step wind input
    r.wind_case_opts    = {
        'U': [8],  # choose below-rated wind speed
        'TMax': 300,
        }
    r.case_inputs = control_base_case
    r.case_inputs[("ServoDyn","Ptch_Cntrl")] = {'vals':[1], 'group':0}  # Individual pitch control must be enabled in ServoDyn
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.n_cores = 5
    r.run_FAST()

    # # Check AWC here
    # filenames = [os.path.join(run_dir,'IEA15MW/simp_step/base/IEA15MW_0.outb')]
    # fast_out = output_processing.output_processing()

    # # Load and plot
    # fastout = fast_out.load_fast_out(filenames)
    # offset_2 = fastout[0]['BldPitch2'] - fastout[0]['BldPitch1']
    # offset_3 = fastout[0]['BldPitch3'] - fastout[0]['BldPitch1']

    # # check that offset (min,max) is very close to prescribed values
    # np.testing.assert_almost_equal(offset_2.max(),pitch2_offset,decimal=3)
    # np.testing.assert_almost_equal(offset_2.min(),pitch2_offset,decimal=3)
    # np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=3)
    # np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=3)



if __name__=="__main__":
    main()
