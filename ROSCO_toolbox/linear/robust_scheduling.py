'''
Methods for finding robust gain schedules
'''

import scipy as sp
import numpy as np
import glob
import os
import yaml
import openmdao.api as om
import warnings
import matplotlib.pyplot as plt
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
# from ROSCO_toolbox.linear import pc_closedloop, pc_openloop, pc_sensitivity, interp_plant, interp_pitch_controller, add_pcomp, feedback
from ROSCO_toolbox.linear.linear_models import LinearTurbineModel, pc_closedloop, pc_openloop, pc_sensitivity, interp_plant, interp_pitch_controller, add_pcomp

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
        self.load_linturb(linturb_options['linfile_root'])
        self.linturb.trim_system(desInputs=['collective'], desOutputs=['RtSpeed'])

        # Load Controller
        self.load_ROSCO(ROSCO_options['path_params'],
                        ROSCO_options['turbine_params'], 
                        ROSCO_options['controller_params'])

    def compute(self, inputs, outputs):
        
        k_float = inputs['k_float']
        if k_float:
            self.linturb = add_pcomp(self.linturb, k_float)

        sm = self.sensitivity_const(inputs['u_eval'], inputs['omega'])
        omega = inputs['omega']

        # Outputs
        outputs['sm']        = sm
        outputs['omega_opt'] = omega

    def load_linturb(self, linfile_root):
        # Parse openfast linearization filenames
        filenames = glob.glob(os.path.join(linfile_root, '*.lin'))
        linfiles = [os.path.split(file)[1] for file in filenames]
        linroots = np.sort(np.unique([file.split('.')[0] for file in linfiles])).tolist()
        linfile_numbers = set([int(file.split('.')[1]) for file in linfiles])
        # Load linturb
        self.linturb = LinearTurbineModel(linfile_root, linroots,
                                     nlin=max(linfile_numbers), rm_hydro=True)

    def load_ROSCO(self, path_params, turbine_params, controller_params):
        self.turbine = ROSCO_turbine.Turbine(turbine_params)
        self.controller = ROSCO_controller.Controller(controller_params)

        # Load turbine, tune controller
        self.turbine.load_from_fast(path_params['FAST_InputFile'], path_params['FAST_directory'])
        self.controller.tune_controller(self.turbine)


    def sensitivity_const(self, u_eval, omega):
        # try:
        #     k_float = inputs[1]
        # except:
        k_float = False

        self.controller.omega_pc = omega
        self.controller.tune_controller(self.turbine)
        if k_float:
            linturb = self.add_pcomp(self.linturb, k_float)
        else:
            linturb = self.linturb
        sens_sys = pc_sensitivity(linturb, self.controller, u_eval)
        ol_sys = pc_openloop(linturb, self.controller, u_eval)
        sm = self.sens_margin(ol_sys, sens_sys)

        return sm

    @staticmethod
    def sens_margin(ol_sys, sens_sys):
        sp_plant = sp.signal.StateSpace(ol_sys.A, ol_sys.B, ol_sys.C, ol_sys.D)
        sp_sens = sp.signal.StateSpace(sens_sys.A, sens_sys.B, sens_sys.C, sens_sys.D)

        def nyquist_min(om): return np.abs(sp.signal.freqresp(sp_plant, w=om)[1] + 1.)
        def sens_max(om): return -np.abs(sp.signal.freqresp(sp_sens, w=om)[1])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            ws, Hs = sp.signal.freqresp(sp_sens)
            # , 0.05, method='COBYLA', tol=1e-5) #, bounds=[0.0,100.0], method='Bounded')
            res = sp.optimize.minimize(nyquist_min, ws[np.argmax(Hs)])

        if any(sp_sens.poles > 0):
            sm = -res.fun
        else:
            sm = res.fun

        return sm

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

if __name__ == '__main__':
    tune()
