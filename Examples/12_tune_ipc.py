"""
12_tune_ipc
--------------
Load a turbine, tune a controller with IPC
In this example:

* Load a turbine from OpenFAST
* Tune a controller with IPC
* Run simple simulation with open loop control
"""

# Python Modules
import os
import matplotlib.pyplot as plt

# ROSCO toolbox modules 
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl

def main():
  this_dir          = os.path.dirname(os.path.abspath(__file__))
  rosco_dir         = os.path.dirname(this_dir)
  example_out_dir   = os.path.join(this_dir,'examples_out')
  example_name      = '12_ipc_sim'
  run_dir = os.path.join(example_out_dir, example_name)

  # Load yaml file (Open Loop Case)
  parameter_filename = os.path.join(this_dir,'Tune_Cases/NREL2p8.yaml')

  case_inputs = {}
  case_inputs[('ServoDyn','Ptch_Cntrl')]   = {'vals': [1], 'group': 0}
  case_inputs[('DISCON_in','IPC_SatMode')]   = {'vals': [0,1,2,3], 'group': 1}
  

  # simulation set up
  r = run_FAST_ROSCO()
  r.tuning_yaml   = parameter_filename
  r.wind_case_fcn = cl.ramp  # single step wind input
  r.wind_case_opts    = {
      'U_start': 11,  # from 10 to 15 m/s
      'U_end': 9,
      't_start': 100,
      't_end': 400,
      'both_dir': True,
      'vert_shear': 0.2
      }
  r.case_inputs = case_inputs
  r.save_dir      = run_dir
  r.rosco_dir     = rosco_dir
  r.n_cores = 4
  r.run_FAST()


  # #  Define Plot cases 
  cases = {}
  cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'RootMyc1', 'RotSpeed']

  out_files = [os.path.join(example_out_dir,example_name,f'NREL2p8/ramp/base/NREL2p8_{i_case}.outb') for i_case in range(4)]
  op = output_processing.output_processing()
  fastout = op.load_fast_out(out_files, tmin=0)
  fig, ax = op.plot_fast_out(cases=cases,showplot=False)
  if False:
    plt.show()
  else:
    fig[0].savefig(os.path.join(example_out_dir,'12_ipc_FAST_Out.png'))

if __name__=="__main__":
   main()

