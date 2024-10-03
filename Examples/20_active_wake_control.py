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

# General Implementation note: AWC strategies will be compromised if the AWC pitch command attempts to lower the blade pitch below value PC_MinPit as specified 
# in the DISCON file, so PC_MinPit may need to be reduced by the user.

# References:
# [1] - Batchelor, G. K., and A. E. Gill. "Analysis of the stability of axisymmetric jets." Journal of fluid mechanics 14.4 (1962): 529-551.
# [2] - Frederik, Joeri A., et al. "The helix approach: Using dynamic individual pitch control to enhance wake mixing in wind farms." Wind Energy 23.8 (2020): 1739-1751.


import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.utilities import read_DISCON #, DISCON_dict
#import numpy as np

def main():
    # Choose your implementation method
    AWC_Mode 			= 1 		# 1 for SNL implementation, 2 for Coleman Transformation implementation


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
        control_base_case[('DISCON_in','AWC_harmonic')] = {'vals': [[0],[0],[1,1],[1,1],[1,1], [1,1]], 'group': 2}
        control_base_case[('DISCON_in','AWC_freq')] = {'vals': [[0],[0.05],[0.05,0.05],[0.05,0.05],[0.05,0.05], [0.05,0.05]], 'group': 2}
        control_base_case[('DISCON_in','AWC_amp')] = {'vals': [[0],[2.5],[2.5,2.5],[2.5,2.5],[2.5,0.0], [0.0,2.5]], 'group': 2}
        control_base_case[('DISCON_in','AWC_clockangle')] = {'vals': [[0],[0],[0,-90],[0,90],[0,0], [180,180]], 'group': 2}
    
    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve  # single step wind input
    r.wind_case_opts    = {
        'U': [14],  # from 10 to 15 m/s
        'TMax': 100,
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
