'''
Methods for finding robust gain schedules
- Implemented in the OpenMDAO framework
'''

import numpy as np
import glob
import os
import openmdao.api as om
import matplotlib.pyplot as plt
import pandas as pd
import multiprocessing as mp
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.linear.linear_models import LinearTurbineModel
from rosco.toolbox.linear.lin_util import add_pcomp, smargin
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.utilities import list_check


class RobustScheduling(om.ExplicitComponent):
    'Finding Robust gain schedules for pitch controllers in FOWTs'

    def initialize(self):
        self.options.declare('linturb_options')
        self.options.declare('ROSCO_options')

    def setup(self):
        # Options
        linturb_options = self.options['linturb_options']
        ROSCO_options = self.options['ROSCO_options']
        ROSCO_options['controller_params']['omega_pc'] = list_check(
            ROSCO_options['controller_params']['omega_pc'], return_bool=False)
        ROSCO_options['controller_params']['zeta_pc'] = list_check(
            ROSCO_options['controller_params']['zeta_pc'], return_bool=False)
        if list_check(ROSCO_options['controller_params']['omega_pc']) or \
                list_check(ROSCO_options['controller_params']['zeta_pc']):
            raise AttributeError(
                'Error: omega_pc and zeta_pc must be scalars for robust controller tuning.')

        # Load ROSCO Turbine and Controller
        if 'dict_inputs' in ROSCO_options.keys():  # Allow for turbine parameters to be passed in as a dictionary (from WEIS)
            dict_inputs = ROSCO_options['dict_inputs']
            # Define turbine based on inputs
            self.turbine = type('', (), {})()
            self.turbine.v_min = float(dict_inputs['v_min'])
            self.turbine.J = float(dict_inputs['rotor_inertia'])
            self.turbine.rho = float(dict_inputs['rho'])
            self.turbine.rotor_radius = float(dict_inputs['R'])
            self.turbine.Ng = float(dict_inputs['gear_ratio'])
            # Incoming value already has gearbox eff included, so have to separate it out
            self.turbine.GenEff = float(
                dict_inputs['generator_efficiency']/dict_inputs['gearbox_efficiency']) * 100.
            self.turbine.GBoxEff = float(dict_inputs['gearbox_efficiency']) * 100.
            self.turbine.rated_rotor_speed = float(dict_inputs['rated_rotor_speed'])
            self.turbine.rated_power = float(dict_inputs['rated_power'])
            self.turbine.rated_torque = float(
                dict_inputs['rated_torque']) / self.turbine.Ng * float(dict_inputs['gearbox_efficiency'])
            self.turbine.v_rated = float(
                dict_inputs['rated_rotor_speed'])*float(dict_inputs['R']) / float(dict_inputs['tsr_operational'])
            self.turbine.v_min = float(dict_inputs['v_min'])
            self.turbine.v_max = float(dict_inputs['v_max'])
            self.turbine.max_pitch_rate = float(dict_inputs['max_pitch_rate'])
            self.turbine.TSR_operational = float(dict_inputs['tsr_operational'])
            self.turbine.max_torque_rate = float(dict_inputs['max_torque_rate'])
            self.turbine.TowerHt = float(dict_inputs['TowerHt'])
            self.turbine.Cp_table = np.squeeze(dict_inputs['Cp_table'])
            self.turbine.Ct_table = np.squeeze(dict_inputs['Ct_table'])
            self.turbine.Cq_table = np.squeeze(dict_inputs['Cq_table'])
            self.turbine.pitch_initial_rad = dict_inputs['pitch_vector']
            self.turbine.bld_edgewise_freq = float(dict_inputs['edge_freq'])
            self.turbine.TSR_initial = dict_inputs['tsr_vector']
            RotorPerformance = ROSCO_turbine.RotorPerformance
            self.turbine.Cp = RotorPerformance(
                self.turbine.Cp_table, self.turbine.pitch_initial_rad, self.turbine.TSR_initial)
            self.turbine.Ct = RotorPerformance(
                self.turbine.Ct_table, self.turbine.pitch_initial_rad, self.turbine.TSR_initial)
            self.turbine.Cq = RotorPerformance(
                self.turbine.Cq_table, self.turbine.pitch_initial_rad, self.turbine.TSR_initial)

            self.controller = ROSCO_controller.Controller(ROSCO_options['controller_params'])
            self.controller.tune_controller(self.turbine)
        else: # otherwise define controller and turbine objection 
            self.controller, self.turbine = load_ROSCO(ROSCO_options['path_params'],
                                                       ROSCO_options['turbine_params'],
                                                       ROSCO_options['controller_params'])
        # Load linear turbine models and trim them
        self.linturb = load_linturb(
            linturb_options['linfile_path'], load_parallel=linturb_options['load_parallel'])
        self.linturb.trim_system(desInputs=['collective'], desOutputs=['RtSpeed'])

        # Inputs
        self.add_input('u_eval',    val=11.,  units='m/s',
                       desc='Wind speeds to evaluate gain schedule')
        self.add_input('omega',     val=0.1,  units='rad/s',   desc='Controller bandwidth')
        self.add_input('k_float',   val=self.controller.Kp_float,
                       units='s',       desc='Floating feedback gain')

        # Outputs
        self.add_output('sm',        val=0.0,                       desc='Stability Margin')
        self.add_output('omega_opt', val=0.01,      units='rad/s',
                        desc='Maximized controller bandwidth')

    def setup_partials(self):
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        '''
        Computes the stability margin for a given controller bandwidth (omega)
        '''
        k_float = inputs['k_float'][0]
        if k_float:
            linturb = add_pcomp(self.linturb, k_float)
        else:
            linturb = self.linturb

        self.controller.omega_pc = inputs['omega'][0]
        self.controller.tune_controller(self.turbine)
        sm = smargin(linturb, self.controller, inputs['u_eval'][0])

        print('omega = {}, sm = {}'.format(inputs['omega'][0], sm))
        omega = inputs['omega']

        # Outputs
        outputs['sm'] = sm
        outputs['omega_opt'] = omega


class rsched_driver():
    '''
    A driver for scheduling robust controllers. 
    Wrapper for the RobustScheduling OpenMDAO Explicit Component

    - Example 12 shows how to use this
    '''

    def __init__(self, options):
        # initialize options
        self.linturb_options = options['linturb_options']
        self.ROSCO_options = options['ROSCO_options']
        self.path_options = options['path_options']
        self.opt_options = options['opt_options']
        self.output_dir = options['path_options']['output_dir']
        self.output_name = options['path_options']['output_name']

        # Output options
        self.output_dir = self.path_options['output_dir']
        self.output_name = self.path_options['output_name']

        # setup levels, especially useful for DOEs
        if 'levels' in self.opt_options.keys():
            self.opt_options['levels'] = self.opt_options['levels']
        else:
            if self.opt_options['driver'] == 'design_of_experiments':
                self.opt_options['levels'] = 20
            elif self.opt_options['driver'] == 'optimization':
                self.opt_options['levels'] = 10

        # Clarify up input sizes
        self.opt_options['omega'] = list_check(self.opt_options['omega'], return_bool=False)
        self.opt_options['k_float'] = list_check(self.opt_options['k_float'], return_bool=False)

    def setup(self):
        '''
        Setup the OpenMDAO problem
        '''
        # Initialize problem
        from openmdao.utils.mpi import MPI, FakeComm
        if MPI:
            self.om_problem = om.Problem(comm=FakeComm())
            self.om_problem._run_root_only = True
            self.om_problem.comm.allgather = MPI.COMM_WORLD.allgather
            self.om_problem.comm.Bcast = MPI.COMM_WORLD.Bcast
        else:
            self.om_problem = om.Problem()

        # Add robust scheduling subsystem
        self.om_problem.model.add_subsystem('r_sched', RobustScheduling(linturb_options=self.linturb_options,
                                                                        ROSCO_options=self.ROSCO_options))

        # Setup design of experiments or optimization problem. 
        # - NJA: DOEs can only be done for one wind speed at a time
        if self.opt_options['driver'] == 'design_of_experiments':
            self.om_problem = self.init_doe(self.om_problem, levels=self.opt_options['levels'])
            if list_check(self.opt_options['windspeed']):
                if len(self.opt_options['windspeed']) == 1:
                    self.opt_options['windspeed'] = self.opt_options['windspeed'][0]
                else:
                    ValueError(
                        'Can only run design of experiments for a single opt_options["windspeed"]')
        elif self.opt_options['driver'] == 'optimization':
            self.om_problem = self.init_optimization(self.om_problem)
        else:
            ValueError(
                "self.opt_options['driver'] must be either 'design_of_experiments' or 'optimization'.")

        # Add stability margin constraints
        self.om_problem.model.add_constraint(
            'r_sched.sm', lower=self.opt_options['stability_margin'])

        # Define objective
        self.om_problem.model.add_objective('r_sched.omega_opt', scaler=-1)

        # Setup OM Problem
        self.om_problem.setup()
        # Set constant values
        if not list_check(self.opt_options['omega']):
            self.om_problem.set_val('r_sched.omega', self.opt_options['omega'])
        if not list_check(self.opt_options['k_float']):
            self.om_problem.set_val('r_sched.k_float', self.opt_options['k_float'])

        # Designate specific problem objects, add design variables
        if self.opt_options['driver'] == 'design_of_experiments':
            self.om_doe = self.om_problem
            self.om_doe = self.add_dv(self.om_doe, ['omega', 'k_float'])
        if self.opt_options['driver'] == 'optimization':
            self.om_opt = self.om_problem
            self.om_opt = self.add_dv(self.om_opt, ['omega', 'k_float'])

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

            # Iterate optimization over each wind speed in opt_options
            for u in self.opt_options['windspeed']:
                om0 = 0.1 # Initial omega - hard coded
                try_count = 0
                while try_count < 3:    # Allow this to try three times before failing. NJA: sometimes small initial condition changes can improve convergence
                    # Setup optimization
                    self.om_opt.set_val('r_sched.u_eval', u)
                    self.om_opt.set_val('r_sched.omega', om0)

                    # Run optimization
                    print('Finding ROSCO tuning parameters for u = {}, sm = {}, omega_pc = {}, k_float = {}'.format(
                        u, self.opt_options['stability_margin'], self.opt_options['omega'], self.opt_options['k_float']))
                    opt_logfile = os.path.join(
                        self.output_dir, self.output_name + '.' + str(u) + ".opt.sql")
                    self.om_opt = self.setup_recorder(self.om_opt, opt_logfile)
                    self.om_opt.run_driver()
                    self.om_opt.cleanup()

                    if self.om_opt.driver.fail:
                        # Restart with a new initial omega if the optimizer failed
                        try_count += 1
                        om0 = np.random.random_sample(1)*self.opt_options['omega'][-1]
                    else:
                        try_count += 1

                # save values
                self.omegas.append(self.om_opt.get_val('r_sched.omega')[0])
                self.k_floats.append(self.om_opt.get_val('r_sched.k_float')[0])
                self.sms.append(self.om_opt.get_val('r_sched.sm')[0])

    def plot_schedule(self):
        '''
        Plots tuning value and stability margins w.r.t. wind speed
        '''
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
        '''
        Add design variables to OM problem

        Parameters:
        -----------
        om_problem: om.problem
            RobustScheduling OpenMDAO problem 
        opt_vars: list
            Variables to optimize, e.g.:['omega', 'k_float']
        
        '''

        if 'omega' in opt_vars and list_check(self.opt_options['omega']):
            om_problem.model.add_design_var(
                'r_sched.omega', lower=self.opt_options['omega'][0], upper=self.opt_options['omega'][1])

        if 'k_float' in opt_vars and list_check(self.opt_options['k_float']):
            om_problem.model.add_design_var(
                'r_sched.k_float', lower=self.opt_options['k_float'][0], upper=self.opt_options['k_float'][1], ref=100)

        # Make sure design variables are stored appropriately in OM problem
        #   - NJA: This is mostly needed for WEIS integration as a nested OpenMDAO problem
        om_problem.model._design_vars = om_problem.model._static_design_vars

        return om_problem

    def init_optimization(self, om_problem):
        'Initialize optimizer'
        om_problem.driver = om.ScipyOptimizeDriver()
        om_problem.driver.options['optimizer'] = 'SLSQP'
        om_problem.driver.options['tol'] = 1e-3
        om_problem.driver.options['maxiter'] = 20
        # om_problem.driver.options['debug_print'] = ['desvars', 'nl_cons']
        om_problem.model.approx_totals(method="fd", step=1e-1, form='central', step_calc='rel')

        return om_problem

    def init_doe(self, om_problem, levels=20):
        '''Initialize DOE driver'''
        om_problem.driver = om.DOEDriver(om.FullFactorialGenerator(levels=levels))
        # om_problem.driver = om.DOEDriver(om.LatinHypercubeGenerator(samples=levels))
        # om_problem.driver = om.DOEDriver(om.UniformGenerator(num_samples=20))
        os.makedirs(self.output_dir, exist_ok=True)
        return om_problem

    @staticmethod
    def setup_recorder(problem, sql_filename):
        ''' Used to prevent memory issues with OM sqlite recorder'''
        recorder = om.SqliteRecorder(sql_filename)

        try:  # Try to remove previous recorder
            problem.driver._recorders.pop()
        except:  # Must be first pass or optimization run
            pass

        problem.driver.add_recorder(recorder)

        try:  # try to re-run recorder setup
            problem._setup_recording()
            problem.driver._setup_recording()
        except:  # Must be first pass
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
    '''
    Loads and processes doe log files
    - OpenMDAO DOEs generate a large set of log files, this function collects them
    
    Parameters:
    -----------
    doe_logs: str,list
        string (single) or list (multiple) of doe log file names
    outfile_name: str, optional
        name of output .csv file to save data

    Returns:
    --------
    df: pd.DataFrame
        Pandas dataframe containing collected DOE data

    '''
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
    '''load OpenMDAO sql file'''
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


def load_linturb(linfile_path, load_parallel=False):
    '''
    Load linear turbine models
    
    Parameters:
    -----------
    linfile_path: string
        Path to folder containing .lin files
    load_parllel: bool
        Load parallel True/False
    '''
    # Parse openfast linearization filenames
    filenames = glob.glob(os.path.join(linfile_path, '*.lin'))
    linfiles = [os.path.split(file)[1] for file in filenames]
    linroots = np.sort(np.unique([file.split('.')[0] for file in linfiles])).tolist()
    linfile_numbers = set([int(file.split('.')[1]) for file in linfiles])
    # Load linturb
    linturb = LinearTurbineModel(linfile_path, linroots,
                                 nlin=max(linfile_numbers), rm_hydro=True, load_parallel=load_parallel)

    return linturb


def load_ROSCO(path_params, turbine_params, controller_params):
    '''
    Load ROSCO controller and turbine objects
    
    Parameters:
    -----------
    path_params: dict
        Path parameters from tuning yaml
    turbine_params: dict
        Turbine parameters from tuning yaml
    controller_params: dict
        Controller parameters from tuning yaml
    '''
    turbine = ROSCO_turbine.Turbine(turbine_params)
    controller = ROSCO_controller.Controller(controller_params)

    # Load turbine, tune controller
    turbine.load_from_fast(path_params['FAST_InputFile'], path_params['FAST_directory'])
    controller.tune_controller(turbine)

    return controller, turbine


if __name__ == '__main__':
    # Setup linear turbine paths
    linfile_path = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), 'Test_Cases', 'IEA-15-240-RWT-UMaineSemi', 'linearizations')
    load_parallel = True
    linturb_options = {'linfile_path': linfile_path,
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
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_out')
    output_name = 'test'
    path_options = {'output_dir': output_dir,
                    'output_name': output_name
                    }

    # Scheduling options
    opt_options = {'driver': 'optimization',  # 'design_of_experiments',
                   'windspeed': [12, 13, 14],  # , 17, 18, 19, 20, 21, 22, 23, 24, 25],
                   'stability_margin': 0.1,
                   'omega': [0.05, 0.2],
                   'k_float': [0.0]}

    options = {}
    options['linturb_options'] = linturb_options
    options['ROSCO_options'] = ROSCO_options
    options['path_options'] = path_options
    options['opt_options'] = opt_options

    sd = rsched_driver(options)
    sd.setup()
    sd.execute()
    if opt_options['driver'] == 'design_of_experiments':
        sd.post_doe(save_csv=True)
    else:
        sd.plot_schedule()

if __name__ == '__main__':
    pass
