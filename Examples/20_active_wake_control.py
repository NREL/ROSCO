'''
----------- 20_active_wake_control ------------
Run openfast with ROSCO and active wake control
-----------------------------------------------

Set up and run simulation with AWC, check outputs

Active wake control with blade pitching is implemented in this example with an adaptation into the rotating frame of the mathematical framework from the classical theory for stability of axisymmetric jets [1], which offers flexibility in specifying the forcing strategy.

The inputs to the controller are:
	-AWC_NumModes	(-)	-> number of forcing modes
	-AWC_n		(-)	-> azimuthal mode number(s) (i.e., the azimuthal mode number relates to the number and direction of the lobes of the wake structure)
	-AWC_clockangle (deg)	-> clocking angle of forcing mode(s)
	-AWC_omega 	(rad/s)	-> frequency(s) of forcing mode(s)
	-AWC_amp 	(rad)	-> pitch amplitude(s) of forcing mode(s)

The latter two inputs may be specified based on the expected inflow while the former three inputs determine the type of active wake control to be used.

Readers may be familiar with several forcing strategies from literature on active wake control that can be represented as follows:
	-collective dynamic induction control: 	AWC_NumModes = 1, AWC_n = 0, AWC_clockangle = 0
	-helix clockwise [2]:			AWC_NumModes = 1, AWC_n = 1, AWC_clockangle = 0
	-helix counter-clockwise [2]:		AWC_NumModes = 1, AWC_n = -1, AWC_clockangle = 0
	-up-and-down:				AWC_NumModes = 2, AWC_n = -1 1, AWC_clockangle = 0 0
	-side-to-side:				AWC_NumModes = 2, AWC_n = -1 1, AWC_clockangle = 90 90
	-other:					Higher-order modes or different combinations of the above can also be specified

	Calculation methodology:
		For each blade, we compute the total phase angle of blade pitch excursion according to:
			AWC_angle(t) = AWC_omega * t - AWC_n * (psi(t) + phi + AWC_clockangle*PI/180)								(eq 1)
			where 	t is time
				phi(t) is the angular offset of the given blade in the rotor plane relative to blade 1
				psi is the angle of blade 1 in the rotor plane from top-dead center
				
		Next, the phase angle is converted into the complex pitch amplitude:
			AWC_complexangle(t) = AWC_amp * EXP(i * AWC_angle(t))											(eq 2)
			where 	i is the square root of -1 
			
		Note that if AWC_NumModes>1, then eq 1 and 2 are computed for each additional mode, and AWC_complexangle becomes a summation over all modes for each blade.

		Finally, the real pitch amplitude, theta(t), to be passed to the next step of the controller is calculated:
			theta(t) = theta_0(t) + REAL(AWC_complexangle(t))											(eq 3)
			where 	theta_0(t) is the controller's nominal pitch command
		
	Rearranging for ease of viewing:		
		Inserting eq 1 into eq 2, and then putting that result into eq 3 gives:
			theta(t) = theta_0(t) + REAL(AWC_amp * EXP(i * (AWC_omega * t - AWC_n * (psi(t) + phi + AWC_clockangle*PI/180))))			(eq 4)
		
		Applying Euler's formula and carrying out the REAL operator:
			theta(t) = theta_0(t) + AWC_amp * cos(AWC_omega * t - AWC_n * (psi(t) + phi + AWC_clockangle*PI/180))					(eq 5)

	As an example, we can set parameters to produce the counter-clockwise helix pattern from [2] using AWC_NumModes = 1, AWC_n = -1, and AWC_clockangle = 0:
		For blade 1, eq 5 becomes:
			theta(t) = theta_0(t) + AWC_amp * cos(AWC_omega * t + psi(t))										(eq 6)		

Note that the inverse multi-blade coordinate (MBC) transformation can also be used to obtain the same result as eq 6.
	Beginning with Eq. 3 from [2], we have 

		/            \			 /               \
		| theta_1(t) |                	 | theta_0(t)    |
		| theta_2(t) | = T^-1(psi(t)) *  | theta_tilt(t) |												(eq 7)
		| theta_3(t) |               	 | theta_yaw(t)  |
		\            /               	 \               /

		where

				/                     	     	\
		             	| 1 cos(psi_1(t)) sin(psi_1(t)) |
		T^-1(psi(t)) =  | 1 cos(psi_2(t)) sin(psi_2(t)) |
				| 1 cos(psi_3(t)) sin(psi_3(t)) |
				\			        /

	Multiplying the first row of the top matrix (and dropping the subscript of blade 1) yields:
		theta(t) = theta_0(t) + theta_tilt(t)*cos(psi(t)) + theta_yaw(t)*sin(psi(t))									(eq 8)
		
	Setting theta_tilt(t) = AWC_amp * cos(AWC_omega * t) and theta_yaw(t) = -AWC_amp * sin(AWC_omega * t) gives:
		theta(t) = theta_0(t) + (AWC_amp * cos(AWC_omega * t))*cos(psi(t)) - (AWC_amp * sin(AWC_omega * t))*sin(psi(t))					(eq 9)

	Applying a Ptolemy identity gives:
		theta(t) = theta_0(t) + AWC_amp * cos(AWC_omega * t + psi(t))											(eq 10)	
		which is equivlanet to eq 6 above.

References:
[1] - Batchelor, G. K., and A. E. Gill. "Analysis of the stability of axisymmetric jets." Journal of fluid mechanics 14.4 (1962): 529-551.
[2] - Frederik, Joeri A., et al. "The helix approach: Using dynamic individual pitch control to enhance wake mixing in wind farms." Wind Energy 23.8 (2020): 1739-1751.

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.utilities import read_DISCON, DISCON_dict
import numpy as np


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))


def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/NREL2p8.yaml')  # will be dummy and overwritten with SNL DISCON params
    run_dir = os.path.join(example_out_dir,'20_active_wake_control/all_cases')
    os.makedirs(run_dir,exist_ok=True)

    # Read all DISCON inputs
    rosco_vt = read_DISCON(os.path.join(rosco_dir,'Test_Cases','NREL_2p8_127/NREL-2p8-127_DISCON.IN'))

    # Apply all discon variables as case inputs
    control_base_case = {}
    for discon_input in rosco_vt:
        control_base_case[('DISCON_in',discon_input)] = {'vals': [rosco_vt[discon_input]], 'group': 0}

    # Set up AWC cases defined above
    control_base_case[('DISCON_in','AWC_NumModes')] = {'vals': [1,1,1,2,2], 'group': 2}
    control_base_case[('DISCON_in','AWC_n')] = {'vals': [[0],[1],[-1],[-1,1], [-1,1]], 'group': 2}
    control_base_case[('DISCON_in','AWC_omega')] = {'vals': [[0.3142],[0.3142],[0.3142],[0.3142,0.3142], [0.3142,0.3142]], 'group': 2}
    control_base_case[('DISCON_in','AWC_amp')] = {'vals': [[0.0175],[0.0175],[0.0175],[0.0175,0.0175], [0.0175,0.0175]], 'group': 2}
    control_base_case[('DISCON_in','AWC_clockangle')] = {'vals': [[0],[0],[0],[0,0], [90,90]], 'group': 2}
    
    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve  # single step wind input
    r.wind_case_opts    = {
        'U': [6],  # from 10 to 15 m/s
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
