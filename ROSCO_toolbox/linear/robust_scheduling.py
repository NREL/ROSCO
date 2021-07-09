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
        # print('THIS IS RUNNING FOR', inputs['u_eval'])
        # print('Omega =',inputs['omega'])
        k_float = inputs['k_float'][0]
        print('k_float', k_float)
        if k_float:
            linturb = add_pcomp(self.linturb, k_float)
        else:
            linturb = self.linturb

        self.controller.omega_pc = inputs['omega']
        self.controller.tune_controller(self.turbine)
        sm = smargin(linturb, self.controller, inputs['u_eval'])
        # print('sm =',sm)

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

    def setup(self):
        '''
        Setup the OpenMDAO problem
        '''
        # Initialize problem
        self.rsched = om.Problem()

        # Add subsystem
        self.rsched.model.add_subsystem('r_sched', RobustScheduling(linturb_options=linturb_options,
                                                                    ROSCO_options=ROSCO_options))

        if self.opt_options['driver'] == 'design_of_experiments':
            self.init_doe()

        # Set inputs and design variables 
        self.rsched.set_val('r_sched.u_eval', self.opt_options['windspeed'])

        if len(self.opt_options['omega']) == 1:
            self.rsched.set_val('r_sched.omega', self.opt_options['omega'][0])
        elif len(self.opt_options['omega']) == 2:
            self.rsched.model.add_design_var(
                'r_sched.omega', lower=self.opt_options['omega'][0], upper=self.opt_options['omega'][1])
        else:
            raise ValueError('Length of opt_options["omega"] cannot be greater than 2.')

        if len(self.opt_options['k_float']) == 1:
            self.rsched.set_val('r_sched.k_float', self.opt_options['k_float'][0])
        elif len(self.opt_options['k_float']) == 2:
            self.rsched.model.add_design_var(
                'r_sched.k_float', lower=self.opt_options['k_float'][0], upper=self.opt_options['k_float'][1])
        else:
            raise ValueError('Length of opt_options["k_float"] cannot be greater than 2.')

        # Add stability margin constraints
        self.rsched.model.add_constraint('r_sched.sm', lower=self.opt_options['stability_margin'])

        # Define objective
        self.rsched.model.add_objective('r_sched.omega_opt', scaler=-1)

    def execute(self):
        '''
        Execute OpenMDAO
        '''
        self.rsched.run_driver()
        

    def init_doe(self):
        'Initialize DOE driver'
        # self.rsched.driver = om.DOEDriver(om.FullFactorialGenerator(levels=10))
        self.rsched.driver = om.DOEDriver(om.UniformGenerator(num_samples=20))
        os.makedirs(self.output_dir, exist_ok=True)
        self.rsched.driver.add_recorder(om.SqliteRecorder(
            os.path.join(self.output_dir, self.output_name + ".sql")))
        # self.rsched.driver.options['run_parallel'] = True
        # self.rsched.driver.options['procs_per_model'] = 1
        
        # Setup OM Problem
        self.rsched.setup()

    def post_doe(self, save_csv=False):
        # Post process doe
        doe_logs = glob.glob(os.path.join(self.output_dir, self.output_name + '.sql*'))
        
        # write to file
        if save_csv:
            doe_outfile_path = os.path.join(self.output_dir, self.output_name)
        
        df = load_DOE(doe_logs, outfile_path=doe_outfile_path)

        return df

def load_DOE(doe_logs, outfile_path=None):
    outdata = [load_OMsql(log) for log in doe_logs]

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

    df = pd.DataFrame.from_dict(collected_data)

    if outfile_path:
        df.to_csv(outdata_fpath + '.csv', index=False)
        print('Saved {}'.format(outdata_fpath + '.csv'))

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

def tune(options):
    linturb_options = options['linturb_options']
    ROSCO_options = options['ROSCO_options']
    output_dir = options['path_options']['output_dir']
    output_name = options['path_options']['output_name']
    opt_options = options['opt_options']

    os.makedirs(output_dir, exist_ok=True)

    lintune = om.Problem()
    lintune.model.add_subsystem('r_sched', RobustScheduling(linturb_options=linturb_options, 
                                                          ROSCO_options=ROSCO_options
                                                          ))

    # Add initial design variables
    lintune.model.add_design_var('r_sched.omega', lower=0.001, upper=0.2)

    # Add constraints
    lintune.model.add_constraint('r_sched.sm', lower=opt_options['stability_margin'], upper=1.0)

    # Define objective
    lintune.model.add_objective('r_sched.omega_opt', scaler=-1)

    # Setup optimizer
    lintune.driver = om.ScipyOptimizeDriver()
    lintune.driver.options['optimizer'] = 'trust-constr'
    lintune.driver.options['tol'] = 1e-3
    lintune.driver.options['maxiter'] = 20
    lintune.driver.options['debug_print'] = ['desvars', 'nl_cons']
    lintune.model.approx_totals(method="fd", step=1e-2, form='central', step_calc='rel')

    # Setup DOE
    lindoe = copy.deepcopy(lintune)
    lindoe.driver = om.DOEDriver(om.FullFactorialGenerator(levels=10))
    lindoe.driver.add_recorder(om.SqliteRecorder(os.path.join(output_dir, output_name + ".sql")))
    lindoe.setup()

    # add k_float
    lintune.model.add_design_var('r_sched.k_float', lower=0.0, upper=50) #, ref=10)
    lintune.setup()

    omegas = []
    k_floats = []
    sms = []
    for u in opt_options['windspeed']:
        
        # Windspeed
        lindoe.set_val('r_sched.u_eval', u)
        lintune.set_val('r_sched.u_eval', u)

        # Run DOE driveer
        lindoe.run_driver()

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
                            collected_data[key].append(data[key][key_idx][0])
                        except:
                            collected_data[key].append(data[key][key_idx])
                    else:
                        collected_data[key].append(np.array(data[key][key_idx]))

        # Find initial omega for optimization
        doe_df = pd.DataFrame(collected_data)
        doe_df.sort_values('r_sched.omega', inplace=True, ignore_index=True)
        try:
            # om0 = doe_df['r_sched.omega_opt'][np.argwhere(np.isclose(doe_df['r_sched.sm'], opt_options['min_sm'], atol=0.1))[0][0]]
            # om0 = doe_df['r_sched.omega_opt'][np.argwhere(np.isclose(doe_df['r_sched.sm'], 0.0, atol=0.1))[0][0]]

            # Fit DOE
            p = np.polyfit(doe_df['r_sched.omega_opt'],doe_df['r_sched.sm'], deg=7)
            omega_fine = np.arange(min(doe_df['r_sched.omega_opt']),max(doe_df['r_sched.omega_opt']),0.0001)
            sm_fine = np.polyval(p, omega_fine)

            # omc1 = np.argwhere(np.isclose(np.diff(sm_fine), 0, atol=0.0001))[0]
            # omc2 = np.argwhere(np.isclose(np.diff(sm_fine), 0, atol=0.0001))[-1]

            # om0 = omega_fine[omc2] - omega_fine[omc1]

            om0 = doe_df['r_sched.omega_opt'][np.argmin(np.diff(doe_df['r_sched.sm']))]
            print('FOUND AN INITIAL CONDITION', om0)


        except: 
            print('Unable to initialize om0 properly')
            om0 = 0.05
        # try:
        #     omc1 = doe_df['r_sched.omega_opt'][np.argwhere(np.isclose(np.diff(doe_df['r_sched.sm']), 0, atol=0.005))[0][0]]
        # except: 
        #     print('Unable to initialize omc1 properly')
        #     omc1 = 0.001
        # try:
        #     omc2 = doe_df['r_sched.omega_opt'][np.argwhere(np.isclose(np.diff(doe_df['r_sched.sm']), 0, atol=0.005))[1][0]]
        # except: 
        #     print('Unable to initialize properly')
        #     omc2 = 0.2


        # Add design variables
        # lintune.model.add_design_var('r_sched.omega', lower=0.001, upper=0.2)
        # lintune.model.add_design_var('r_sched.omega', lower=omc1, upper=min(omc2,0.2))
        # lintune.model.add_design_var('r_sched.k_float', lower=0.0, upper=20.0)

        # Setup optimization
        lintune.set_val('r_sched.omega', om0)
        lintune.set_val('r_sched.k_float', 11.0)

        # Run optimization
        lintune.driver.add_recorder(om.SqliteRecorder(os.path.join(output_dir, output_name + '.' + str(u) + ".sql")))

        lintune.run_driver()

        # save values
        omegas.append(lintune.get_val('r_sched.omega')[0])
        k_floats.append(lintune.get_val('r_sched.k_float')[0])
        sms.append(lintune.get_val('r_sched.sm')[0])

    fig, ax = plt.subplots(3,1,constrained_layout=True, sharex=True)
    ax[0].plot(opt_options['windspeed'], omegas)
    ax[0].set_ylabel('omega_pc')
    ax[1].plot(opt_options['windspeed'], k_floats)
    ax[1].set_ylabel('k_float')
    ax[2].plot(opt_options['windspeed'], sms)
    ax[2].set_ylabel('stability margin')
    ax[2].set_xlabel('Wind Speed')


    plt.show()


def design_of_experiments(options, save_csv=False):
    # load options
    linturb_options = options['linturb_options']
    ROSCO_options   = options['ROSCO_options']
    opt_options     = options['opt_options']

    # Output options
    output_dir  = path_options['output_dir']
    output_name = path_options['output_name']
    tuning_doe = om.Problem()
    tuning_doe.model.add_subsystem('r_sched', RobustScheduling(linturb_options=linturb_options,
                                                          ROSCO_options=ROSCO_options))

    # Setup optimizer
    tuning_doe.driver = om.DOEDriver(om.FullFactorialGenerator(levels=25))
    os.makedirs(output_dir, exist_ok=True)
    tuning_doe.driver.add_recorder(om.SqliteRecorder(os.path.join(output_dir, output_name + ".sql")))
    # tuning_doe.driver.options['run_parallel'] = True
    # tuning_doe.driver.options['procs_per_model'] = 1

    # # Setup OM Problem
    tuning_doe.setup()


    # -- Set inputs and design variables --
    if len(opt_options['omega']) == 1:
        tuning_doe.set_val('r_sched.u_eval', opt_options['omega'])
    elif len(opt_options['omega']) == 2:
        tuning_doe.model.add_design_var(
            'r_sched.omega', lower=opt_options['omega'][0], upper=opt_options['omega'][1])
    else:
        raise ValueError('Length of opt_options["omega"] cannot be greater than 2.')

    if len(opt_options['k_float']) == 1:
        tuning_doe.set_val('r_sched.k_float', opt_options['k_float'])
    elif len(opt_options['k_float']) == 2:
        tuning_doe.model.add_design_var(
            'r_sched.k_float', lower=opt_options['k_float'][0], upper=opt_options['k_float'][1])
    else:
        raise ValueError('Length of opt_options["k_float"] cannot be greater than 2.')


    # Add constraints
    tuning_doe.model.add_constraint('r_sched.sm', lower=opt_options['stability_margin'])

    # Define objective
    tuning_doe.model.add_objective('r_sched.omega_opt', scaler=-1)

    # # Set Wind Speed
    tuning_doe.set_val('r_sched.u_eval', opt_options['windspeed'])

    # # Run and cleanuip
    tuning_doe.run_driver()
    # tuning_doe.cleanup()

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

    df = pd.DataFrame.from_dict(collected_data)

    # write to file
    if save_csv:
        outdata_fpath = os.path.join(output_dir, output_name)
        df.to_csv(outdata_fpath + '.csv', index=False)
        print('Saved {}'.format(outdata_fpath + '.csv'))


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





if __name__ == '__main__':

    # Setup linear turbine paths
    linfile_root = '/Users/nabbas/Documents/Projects/RobustControl/linearizations/case_outputs/case_4'
    linturb_options = {'linfile_root': linfile_root}

    # ROSCO options
    parameter_filename = '/Users/nabbas/Documents/WindEnergyToolbox/ROSCO/Tune_Cases/IEA15MW.yaml'
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']
    path_params['FAST_directory'] = '/Users/nabbas/Documents/WindEnergyToolbox/ROSCO/Test_Cases/IEA-15-240-RWT-UMaineSemi'
    ROSCO_options = {
        'path_params': path_params,
        'turbine_params': turbine_params,
        'controller_params': controller_params
    }

    # Path options
    output_dir = os.path.join( os.path.dirname(os.path.abspath(__file__)), 'test_out' )
    output_name = 'test_doe'
    path_options = {'output_dir': output_dir,
                    'output_name': output_name
                    }

    # Scheduling options
    opt_options = { 'driver': 'design_of_experiments',
                    'windspeed': 12, #np.arange(12, 18),
                   'stability_margin': 0.1,
                   'omega':[0.05, 0.2],
                   'k_float': [0.0]}
    
    options = {}
    options['linturb_options'] = linturb_options
    options['ROSCO_options']   = ROSCO_options
    options['path_options']    = path_options
    options['opt_options']      = opt_options

    tune(options)
    # design_of_experiments(options)


    # sd = rsched_driver(options)
    # sd.setup()
    # sd.execute()
    # sd.post_doe(save_csv=True)
