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
import copy
import multiprocessing as mp
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.linear.linear_models import LinearTurbineModel
from ROSCO_toolbox.linear.lin_util import add_pcomp, smargin
from ROSCO_toolbox.inputs.validation import load_rosco_yaml

class RobustScheduling(om.ExplicitComponent):
    'Finding Robust gain schedules for pitch controllers in FOWTs'

    def initialize(self):
        self.options.declare('linturb_options')
        self.options.declare('ROSCO_options')

    def setup(self):
        # Options
        linturb_options = self.options['linturb_options']
        ROSCO_options = self.options['ROSCO_options']

        # Load ROSCO Controller
        self.controller, self.turbine  = load_ROSCO(ROSCO_options['path_params'],
                                                        ROSCO_options['turbine_params'], 
                                                        ROSCO_options['controller_params'])
        # Load linear turbine models and trim them
        self.linturb = load_linturb(linturb_options['linfile_root'], load_parallel=linturb_options['load_parallel'])
        self.linturb.trim_system(desInputs=['collective'], desOutputs=['RtSpeed'])

        # Inputs
        self.add_input('u_eval',    val=11.,  units='m/s',     desc='Wind speeds to evaluate gain schedule')
        self.add_input('omega',     val=0.1,  units='rad/s',   desc='Controller bandwidth')
        self.add_input('k_float',   val=self.controller.Kp_float,  units='s',       desc='Floating feedback gain')

        # Outputs
        self.add_output('sm',        val=0.0,                       desc='Stability Margin')
        self.add_output('omega_opt', val=0.01,      units='rad/s', desc='Maximized controller bandwidth')

    def compute(self, inputs, outputs):
        k_float = inputs['k_float'][0]
        if k_float:
            linturb = add_pcomp(self.linturb, k_float)
        else:
            linturb = self.linturb

        self.controller.omega_pc = inputs['omega'][0]
        self.controller.tune_controller(self.turbine)
        sm = smargin(linturb, self.controller, inputs['u_eval'][0])

        omega = inputs['omega']
        
        # Outputs
        outputs['sm']        = sm
        outputs['omega_opt'] = omega

class rsched_driver():
    '''
    A driver for scheduling robust controllers
    '''

    def __init__(self,options):
        self.linturb_options = options['linturb_options']
        self.ROSCO_options = options['ROSCO_options']
        self.path_options = options['path_options']
        self.opt_options = options['opt_options']
        self.output_dir = options['path_options']['output_dir']
        self.output_name = options['path_options']['output_name']

        # Output options
        self.output_dir = self.path_options['output_dir']
        self.output_name = self.path_options['output_name']

        if 'levels' in self.opt_options.keys():
            self.opt_options['levels'] = self.opt_options['levels']
        else:
            if self.opt_options['driver'] == 'design_of_experiments':
                self.opt_options['levels'] = 20
            elif self.opt_options['driver'] == 'optimization':
                self.opt_options['levels'] = 10
    def setup(self):
        '''
        Setup the OpenMDAO problem
        '''
        # Initialize problem
        self.om_problem = om.Problem()

        # Add subsystem
        self.om_problem.model.add_subsystem('r_sched', RobustScheduling(linturb_options=self.linturb_options,
                                                                    ROSCO_options=self.ROSCO_options))

        if self.opt_options['driver'] == 'design_of_experiments':
            self.om_problem = self.init_doe(self.om_problem, levels=self.opt_options['levels'])
            if isinstance(self.opt_options['windspeed'], list):
                if len(self.opt_options['windspeed']) == 1:
                    self.opt_options['windspeed'] = self.opt_options['windspeed'][0]
                else:
                    ValueError('Can only run design of experiments for a single opt_options["windspeed"]')
        elif self.opt_options['driver'] == 'optimization':
            self.om_problem = self.init_doe(self.om_problem, levels=self.opt_options['levels'])
        else:
            ValueError("self.opt_options['driver'] must be either 'design_of_experiments' or 'optimization'.")
            
        # Add stability margin constraints
        self.om_problem.model.add_constraint('r_sched.sm', lower=self.opt_options['stability_margin'])

        # Define objective
        self.om_problem.model.add_objective('r_sched.omega_opt', scaler=-1)

        # Setup OM Problem
        self.om_problem.setup()

        # Set constant values
        if len(self.opt_options['omega']) == 1:
            self.om_problem.set_val('r_sched.omega', self.opt_options['omega'][0])
        if len(self.opt_options['k_float']) == 1:
            self.om_problem.set_val('r_sched.k_float', self.opt_options['k_float'][0])
        
        # Designate specific problem objects, add design variables
        if self.opt_options['driver'] == 'design_of_experiments':
            self.om_doe = self.om_problem
            self.om_doe = self.add_dv(self.om_doe, ['omega', 'k_float'])
        if self.opt_options['driver'] == 'optimization':
            # self.om_doe = copy.deepcopy(self.om_problem)
            # self.om_doe = self.add_dv(self.om_doe, ['omega'])

            self.om_opt = self.om_problem
            self.om_opt = self.add_dv(self.om_opt, ['omega', 'k_float'])
            self.om_opt = self.init_optimization(self.om_opt)
        
    
    def execute(self):
        '''
        Execute OpenMDAO
        '''
        if self.opt_options['driver'] == 'design_of_experiments':
            u = self.opt_options['windspeed']
            self.om_doe.set_val('r_sched.u_eval', u)
            self.doe_logfile = os.path.join(
                self.output_dir, self.output_name + '.' + str(u) + ".doe.sql")
            self.om_doe.driver.add_recorder(om.SqliteRecorder(self.doe_logfile))
            self.om_doe.run_driver()

        elif self.opt_options['driver'] == 'optimization':
            self.omegas = []
            self.k_floats = []
            self.sms = []
            for u in self.opt_options['windspeed']:
                # Run initial doe
                # print('Finding initial condition for u = ', u)
                # self.om_doe.set_val('r_sched.u_eval', u)
                # self.doe_logfile = os.path.join(
                #     self.output_dir, self.output_name + '.' + str(u) + ".doe.sql")
                # self.om_doe = self.setup_recorder(self.om_doe, self.doe_logfile)
                # self.om_doe.run_driver()
                # self.om_doe.cleanup()
            
                # # Load doe
                # doe_df = self.post_doe(save_csv=True)
                # doe_df.sort_values('r_sched.omega', inplace=True, ignore_index=True)

                # try:
                #     # Find initial omega
                #     om0 = np.mean(doe_df['r_sched.omega_opt'][doe_df['r_sched.sm'] > 0.0])
                #     if np.isnan(om0):
                #         raise
                #     print('Found an initial condition:', om0)

                # except:
                #     print('Unable to initialize om0 properly')
                #     om0 = 0.05
                om0 = 0.01
                # Setup optimization
                self.om_opt.set_val('r_sched.u_eval', u)
                self.om_opt.set_val('r_sched.omega', om0)

                # Run optimization
                print('Running optimization for u = ', u)
                opt_logfile = os.path.join(self.output_dir, self.output_name + '.' + str(u) + ".opt.sql")
                self.om_opt = self.setup_recorder(self.om_opt, opt_logfile)
                self.om_opt.run_driver()
                self.om_opt.cleanup()

                # save values
                self.omegas.append(self.om_opt.get_val('r_sched.omega')[0])
                self.k_floats.append(self.om_opt.get_val('r_sched.k_float')[0])
                self.sms.append(self.om_opt.get_val('r_sched.sm')[0])

    def plot_schedule(self):
        fig, ax = plt.subplots(3, 1, constrained_layout=True, sharex=True)
        ax[0].plot(self.opt_options['windspeed'], self.omegas)
        ax[0].set_ylabel('omega_pc')
        ax[0].grid()
        ax[1].plot(self.opt_options['windspeed'], self.k_floats)
        ax[1].set_ylabel('k_float')
        ax[1].grid()
        ax[2].plot(self.opt_options['windspeed'], self.sms)
        ax[2].set_ylabel('stability margin')
        ax[2].set_xlabel('Wind Speed')
        ax[2].grid()
        plt.show()


    def add_dv(self, om_problem, opt_vars):
        '''add design variables'''

        if 'omega' in opt_vars and len(self.opt_options['omega']) == 2:
            om_problem.model.add_design_var(
                    'r_sched.omega', lower=self.opt_options['omega'][0], upper=self.opt_options['omega'][1])

        if 'k_float' in opt_vars and len(self.opt_options['k_float']) == 2:
            om_problem.model.add_design_var(
                'r_sched.k_float', lower=self.opt_options['k_float'][0], upper=self.opt_options['k_float'][1], ref=100)

        # Make sure design variables are stored appropriately in OM problem
        om_problem.model._design_vars = om_problem.model._static_design_vars

        return om_problem

    def init_optimization(self, om_problem):
        'Initialize optimizer'
        om_problem.driver = om.ScipyOptimizeDriver()
        om_problem.driver.options['optimizer'] = 'SLSQP'
        om_problem.driver.options['tol'] = 1e-3
        om_problem.driver.options['maxiter'] = 20
        om_problem.driver.options['debug_print'] = ['desvars', 'nl_cons']
        om_problem.model.approx_totals(method="fd", step=1e-1, form='central', step_calc='rel')

        return om_problem

    def init_doe(self, om_problem, levels=20):
        '''Initialize DOE driver'''
        om_problem.driver = om.DOEDriver(om.FullFactorialGenerator(levels=levels))
        # om_problem.driver = om.DOEDriver(om.UniformGenerator(num_samples=20))
        os.makedirs(self.output_dir, exist_ok=True)
        # om_problem.driver.options['run_parallel'] = True
        # om_problem.driver.options['procs_per_model'] = 1
    
        return om_problem

    @staticmethod
    def setup_recorder(problem, sql_filename):
        ''' Used to prevent memory issues with OM sqlite recorder'''
        recorder = om.SqliteRecorder(sql_filename)
        
        try: # Try to remove previous recorder
            problem.driver._recorders.pop()
        except: # Must be first pass or optimization run
            pass

        problem.driver.add_recorder(recorder)
        
        try: # try to re-run recorder setup 
            problem._setup_recording()
            problem.driver._setup_recording()
        except: # Must be first pass
            pass

        return problem

    def post_doe(self, save_csv=False):
        '''Post process doe'''
        # write to file
        if save_csv:
            doe_outfile = '.'.join(self.doe_logfile.split('.')[:-1]) + '.csv'
        else:
            doe_outfile = None
        
        df = load_DOE(self.doe_logfile, outfile_name=doe_outfile)

        return df


def load_DOE(doe_logs, outfile_name=None):
    if isinstance(doe_logs, str):
        doe_logs = [doe_logs]

    if False:
        cores = mp.cpu_count()
        pool = mp.Pool(min(len(doe_logs), cores))

        # load sql file
        outdata = pool.map(load_OMsql, doe_logs)
        pool.close()
        pool.join()
    # no multiprocessing
    else:
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

    df = pd.DataFrame.from_dict(collected_data, dtype=float)

    if outfile_name:
        df.to_csv(outfile_name, index=False)
        print('Saved {}'.format(outfile_name))
    
    return df

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

def load_linturb(linfile_root, load_parallel=False):
    # Parse openfast linearization filenames
    filenames = glob.glob(os.path.join(linfile_root, '*.lin'))
    linfiles = [os.path.split(file)[1] for file in filenames]
    linroots = np.sort(np.unique([file.split('.')[0] for file in linfiles])).tolist()
    linfile_numbers = set([int(file.split('.')[1]) for file in linfiles])
    # Load linturb
    linturb = LinearTurbineModel(linfile_root, linroots,
                                 nlin=max(linfile_numbers), rm_hydro=True, load_parallel=load_parallel)

    return linturb

def load_ROSCO(path_params, turbine_params, controller_params):
    turbine = ROSCO_turbine.Turbine(turbine_params)
    controller = ROSCO_controller.Controller(controller_params)

    # Load turbine, tune controller
    turbine.load_from_fast(path_params['FAST_InputFile'], path_params['FAST_directory'])
    controller.tune_controller(turbine)

    return controller, turbine


if __name__ == '__main__':

    # Setup linear turbine paths
    linfile_root = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), 'Test_Cases', 'IEA-15-240-RWT-UMaineSemi', 'linearizations')
    load_parallel = True
    linturb_options = {'linfile_root': linfile_root,
                       'load_parallel': load_parallel}

    # ROSCO options
    parameter_filename = os.path.join(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'Tune_Cases', 'IEA15MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']
    path_params['FAST_directory'] = os.path.join(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'Test_Cases', 'IEA-15-240-RWT-UMaineSemi')
    ROSCO_options = {
        'path_params': path_params,
        'turbine_params': turbine_params,
        'controller_params': controller_params
    }

    # Path options
    output_dir = os.path.join( os.path.dirname(os.path.abspath(__file__)), 'test_out' )
    output_name = 'test'
    path_options = {'output_dir': output_dir,
                    'output_name': output_name
                    }

    # Scheduling options
    opt_options = { 'driver': 'optimization', #'design_of_experiments',
                    'windspeed': [12, 13, 14], #, 17, 18, 19, 20, 21, 22, 23, 24, 25],
                   'stability_margin': 0.1,
                   'omega':[0.05, 0.2],
                   'k_float': [0.0]}
    
    options = {}
    options['linturb_options'] = linturb_options
    options['ROSCO_options']   = ROSCO_options
    options['path_options']    = path_options
    options['opt_options']      = opt_options

    sd = rsched_driver(options)
    sd.setup()
    sd.execute()
    if opt_options['driver'] == 'design_of_experiments':
        sd.post_doe(save_csv=True)
    else:
        sd.plot_schedule()

