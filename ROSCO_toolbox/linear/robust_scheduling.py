'''
Methods for finding robust gain schedules
'''

import scipy as sp
import numpy as np
import glob
import os
import yaml
import openmdao.api as om
import matplotlib.pyplot as plt
import pandas as pd
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.linear.linear_models import LinearTurbineModel
from ROSCO_toolbox.linear.lin_util import add_pcomp, smargin


class RobustScheduling(om.ExplicitComponent):
    'Finding Robust gain schedules for pitch controllers in FOWTs'

    def initialize(self):
        self.options.declare('opt_options')
        self.options.declare('linturb_options')
        self.options.declare('ROSCO_options')

    def setup(self):
        # Options
        opt_options = self.options['opt_options']
        linturb_options = self.options['linturb_options']
        ROSCO_options = self.options['ROSCO_options']

        # Inputs
        self.add_input('u_eval',    val=11.,  units='m/s',     desc='Wind speeds to evaluate gain schedule')
        self.add_input('omega',     val=0.1,  units='rad/s',   desc='Controller bandwidth')
        self.add_input('k_float',   val=0.0,  units='s',       desc='Floating feedback gain')

        # Outputs
        self.add_output('sm',        val=0.0,                       desc='Stability Margin')
        self.add_output('omega_opt', val=0.01,      units='rad/s', desc='Maximized controller bandwidth')

        # Load linear turbine models and trim them
        self.linturb = load_linturb(linturb_options['linfile_root'])
        self.linturb.trim_system(desInputs=['collective'], desOutputs=['RtSpeed'])

        # Load Controller
        self.controller, self.turbine  = load_ROSCO(ROSCO_options['path_params'],
                                                        ROSCO_options['turbine_params'], 
                                                        ROSCO_options['controller_params'])

    def compute(self, inputs, outputs):
        
        k_float = inputs['k_float']
        if k_float:
            self.linturb = add_pcomp(self.linturb, k_float)

        self.controller.omega_pc = inputs['omega']
        self.controller.tune_controller(self.turbine)
        sm = smargin(self.linturb, self.controller, inputs['u_eval'])

        omega = inputs['omega']

        # Outputs
        outputs['sm']        = sm
        outputs['omega_opt'] = omega

def load_linturb(linfile_root):
    # Parse openfast linearization filenames
    filenames = glob.glob(os.path.join(linfile_root, '*.lin'))
    linfiles = [os.path.split(file)[1] for file in filenames]
    linroots = np.sort(np.unique([file.split('.')[0] for file in linfiles])).tolist()
    linfile_numbers = set([int(file.split('.')[1]) for file in linfiles])
    # Load linturb
    linturb = LinearTurbineModel(linfile_root, linroots,
                                    nlin=max(linfile_numbers), rm_hydro=True)

    return linturb

def load_ROSCO(path_params, turbine_params, controller_params):
    turbine = ROSCO_turbine.Turbine(turbine_params)
    controller = ROSCO_controller.Controller(controller_params)

    # Load turbine, tune controller
    turbine.load_from_fast(path_params['FAST_InputFile'], path_params['FAST_directory'])
    controller.tune_controller(turbine)

    return controller, turbine


def load_OMsql(log):
    print('loading {}'.format(log))
    cr = om.CaseReader(log)
    rec_data = {}
    driver_cases = cr.list_cases('driver')
    cases = cr.get_cases('driver')
    for case in cases:
        for key in case.outputs.keys():
            if key not in rec_data:
                rec_data[key] = []
            rec_data[key].append(case[key])

    return rec_data

def tune():
    # Linear turbine options
    linfile_root = '/Users/nabbas/Documents/Projects/RobustControl/linearizations/case_outputs/case_4'
    linturb_options = {'linfile_root': linfile_root}

    # ROSCO options
    parameter_filename = '/Users/nabbas/Documents/WindEnergyToolbox/ROSCO_toolbox/Tune_Cases/IEA15MW.yaml'
    inps = yaml.safe_load(open(parameter_filename))
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']
    path_params['FAST_directory'] = '/Users/nabbas/Documents/WindEnergyToolbox/ROSCO_toolbox/Test_Cases/IEA-15-240-RWT-UMaineSemi'
    ROSCO_options = {
        'path_params': path_params,
        'turbine_params': turbine_params,
        'controller_params': controller_params
        }

    opt_options = {'u_eval': 12.0}
    rtune = om.Problem()
    rtune.model.add_subsystem('r_sched', RobustScheduling(linturb_options=linturb_options, 
                                                          ROSCO_options=ROSCO_options, 
                                                          opt_options=opt_options))

    # Setup optimizer
    rtune.driver = om.ScipyOptimizeDriver()
    rtune.driver.options['optimizer'] = 'trust-constr'
    rtune.driver.options['tol'] = 0.01
    rtune.driver.options['maxiter'] = 100
    rtune.driver.options['debug_print'] = ['desvars', 'nl_cons']
    rtune.model.approx_totals(method="fd", step=0.1, form='central')

    # Add design variables
    rtune.model.add_design_var('r_sched.omega', lower=0.01, upper=0.3)
    rtune.model.add_design_var('r_sched.k_float', lower=0.0, upper=20.0)

    # Add constraints
    rtune.model.add_constraint('r_sched.sm', lower=0.1)

    # Define objective
    rtune.model.add_objective('r_sched.omega_opt', scaler=-1)

    # Setup
    rtune.setup()

    u_vec = np.arange(12, 21)
    omegas = []
    k_floats = []
    for u in u_vec:
        # Initial values
        rtune.set_val('r_sched.u_eval', u)
        rtune.set_val('r_sched.omega', 0.1)
        rtune.set_val('r_sched.k_float', 0.0)

        # Run
        rtune.run_driver()

        # save values
        omegas.append(rtune.get_val('r_sched.omega')[0])
        k_floats.append(rtune.get_val('r_sched.k_float')[0])

    fig, ax = plt.subplots(2,1,constrained_layout=True)
    ax[0].plot(u_vec, omegas)
    ax[1].plot(u_vec, k_floats)
    plt.show()


def design_of_experiments():
    # Linear turbine options
    linfile_root = '/Users/nabbas/Documents/Projects/RobustControl/linearizations/case_outputs/case_4'
    linturb_options = {'linfile_root': linfile_root}

    # ROSCO options
    parameter_filename = '/Users/nabbas/Documents/WindEnergyToolbox/ROSCO_toolbox/Tune_Cases/IEA15MW.yaml'
    inps = yaml.safe_load(open(parameter_filename))
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']
    path_params['FAST_directory'] = '/Users/nabbas/Documents/WindEnergyToolbox/ROSCO_toolbox/Test_Cases/IEA-15-240-RWT-UMaineSemi'
    ROSCO_options = {
        'path_params': path_params,
        'turbine_params': turbine_params,
        'controller_params': controller_params
    }

    # Output options
    output_dir = os.path.join( os.path.dirname(os.path.abspath(__file__)), 'doe_out' )
    output_name = 'doe_kfloat0'
    opt_options = {'u_eval': 14.0}
    rtune = om.Problem()
    rtune.model.add_subsystem('r_sched', RobustScheduling(linturb_options=linturb_options,
                                                          ROSCO_options=ROSCO_options,
                                                          opt_options=opt_options))

    # Setup optimizer
    rtune.driver = om.DOEDriver(om.UniformGenerator(num_samples=100))
    os.makedirs(output_dir, exist_ok=True)
    rtune.driver.add_recorder(om.SqliteRecorder(os.path.join(output_dir, output_name + ".sql")))

    # Add design variables
    rtune.model.add_design_var('r_sched.omega', lower=0.01, upper=0.25)
    # rtune.model.add_design_var('r_sched.k_float', lower=0.0, upper=20.0)

    # Add constraints
    rtune.model.add_constraint('r_sched.sm', lower=0.1)

    # Define objective
    rtune.model.add_objective('r_sched.omega_opt', scaler=-1)

    # # Setup OM Problem
    rtune.setup()

    # # Set Wind Speed
    u = 14
    rtune.set_val('r_sched.u_eval', u)
    rtune.set_val('r_sched.k_float', 0.0)

    # # Run and cleanuip
    rtune.run_driver()
    # rtune.cleanup()

    # Post process doe
    doe_logs = glob.glob(os.path.join(output_dir, output_name + '.sql*'))
    outdata = [load_OMsql(log) for log in doe_logs]

    collected_data = {}
    for data in outdata:
        for key in data.keys():
            if key not in collected_data.keys():
                collected_data[key] = []

            for key_idx, _ in enumerate(data[key]):
                if isinstance(data[key][key_idx], int):
                    collected_data[key].append(np.array(data[key][key_idx]))
                elif len(data[key][key_idx]) == 1:
                    try:
                        collected_data[key].append(np.array(data[key][key_idx][0]))
                    except:
                        collected_data[key].append(np.array(data[key][key_idx]))
                else:
                    collected_data[key].append(np.array(data[key][key_idx]))

    df = pd.DataFrame.from_dict(collected_data)

    # write to file
    outdata_fpath = os.path.join(output_dir, output_name)
    df.to_csv(outdata_fpath + '.csv', index=False)
    print('Saved {}'.format(outdata_fpath + '.csv'))

if __name__ == '__main__':
    tune()
    design_of_experiments()
