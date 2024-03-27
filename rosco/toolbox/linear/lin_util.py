'''
Utilities to aid linear model controller development and analysis
'''
import numpy as np
import scipy as sp
import control as co
import warnings
import copy

def pc_openloop(linturb, controller, u):
    '''
    This function creates a set of openloop transfer function for the wind turbine with a pitch controller
    
    Inputs
    ------
    linturb: object
        LinearTurbineModel object
    controller: object
        ROSCO toolbox controller object
    u: float
        wind speed to evaluate system at
    '''
    if linturb.B_ops.shape[1] > 1:
        raise TypeError(
            'OpenLoop system calculations are only supported for single input systems, please run LinearTurbine.trim_system with len(desInputs) = 1')
    if linturb.C_ops.shape[0] > 1:
        raise TypeError(
            'OpenLoop system calculations are only supported for single output systems, please run LinearTurbine.trim_system with len(desOutputs) = 1')

    # Interpolate plant and controller
    P = interp_plant(linturb, u)
    Cs = interp_pitch_controller(controller, u)

    # Combine controller and plant
    sys_ol = Cs*P

    return sys_ol


def pc_closedloop(linturb, controller, u):
    '''
    This function creates a set of closed loop transfer function for the wind turbine with a pitch controller
    
    Inputs
    ------
    linturb: object
        LinearTurbineModel object
    controller: object
        ROSCO toolbox controller object
    u: float
        wind speed to evaluate closed loop system at
    Returns
    -------
    sys_cl: scipy StateSpaceContinuous 
        closed loop system
    '''
    if linturb.B_ops.shape[1] > 1:
        raise TypeError(
            'OpenLoop system calculations are only supported for single input systems, please run LinearTurbine.trim_system with len(desInputs) = 1')
    if linturb.C_ops.shape[0] > 1:
        raise TypeError(
            'OpenLoop system calculations are only supported for single output systems, please run LinearTurbine.trim_system with len(desOutputs) = 1')

    # Interpolate plant and controller
    P = interp_plant(linturb, u)
    Cs = interp_pitch_controller(controller, u)

    # Combine controller and plant
    sys_cl = feedback(Cs*P, 1)

    return sys_cl


def pc_sensitivity(linturb, controller, u):
    '''
    This function finds the sensitivity function for the wind turbine with a pitch controller
    
    Inputs
    ------
    linturb: object
        LinearTurbineModel object
    controller: object
        ROSCO toolbox controller object
    u: float
        wind speed to evaluate system at
    '''

    if linturb.B_ops.shape[1] > 1:
        raise TypeError(
            'OpenLoop system calculations are only supported for single input systems, please run LinearTurbine.trim_system with len(desInputs) = 1')
    if linturb.C_ops.shape[0] > 1:
        raise TypeError(
            'OpenLoop system calculations are only supported for single output systems, please run LinearTurbine.trim_system with len(desOutputs) = 1')

    # Interpolate plant and controller
    P = interp_plant(linturb, u)
    Cs = interp_pitch_controller(controller, u)

    # Calculate sensitivity function
    sens = feedback(1, Cs*P)

    return sens

def smargin(linturb, controller, u_eval):
    '''
    Calculates the stability margin for an open-loop system

    linturb: object
        LinearTurbineModel object
    controller: object
        ROSCO toolbox controller object
    u_eval: float
        wind speed to evaluate system at
    '''

    # Find standard transfer functions
    sens_sys = pc_sensitivity(linturb, controller, u_eval)
    ol_sys = pc_openloop(linturb, controller, u_eval)
    # Convert to state space
    sp_plant = sp.signal.StateSpace(ol_sys.A, ol_sys.B, ol_sys.C, ol_sys.D)
    sp_sens = sp.signal.StateSpace(sens_sys.A, sens_sys.B, sens_sys.C, sens_sys.D)

    # Minimum distance to the critical point on the nyquist diagram
    def nyquist_min(om): return np.abs(sp.signal.freqresp(sp_plant, w=om)[1] + 1.)
    # Maximum value of sensitivity function
    def sens_min(om): return -sp.signal.bode(sp_sens, w=om)[1]

    with warnings.catch_warnings():
        warnings.simplefilter("ignore") # NJA: Lots of scipy errors because of poorly posed state matrices - ignore them
        
        # Find first local maximum in bode magnitude of the sensitivity function
        ws, m, _ = sp.signal.bode(sp_sens, n=100000)
        m0 = m[0]
        m1 = m[1]
        i = 1
        w0 = ws[0]
        while m1 >= m0 and i < len(ws)-1:
            m0 = m1
            m1 = m[i+1]
            i += 1
            w0 = ws[i]

        # --- Find important magnitudes and related frequencies with just the sampled values ---
        #    # NJA - optimization is run in the next "step" to fine tune these values further
        # magnitude of sensitivity function at first local maximum
        sm_mag = sp.signal.freqresp(sp_plant, w=w0)[1]
        sm = np.sqrt((1 - np.abs(sm_mag.real))**2 + sm_mag.imag**2)
        # magnitude and frequency of distance to critical point on nyquist
        nearest_nyquist = nyquist_min(ws).min()
        nearest_nyquist_freq = ws[nyquist_min(ws).argmin()]
        mag_at_min = sp.signal.freqresp(sp_plant, w=nearest_nyquist_freq)[1]

        # If any poles of the sensitivity function are unstable, solve an optimization problem on the Nyquist trajectory
        # to minimize the distance to the critical point over all frequencies. Start the gradient-based optimization at
        # the nearest value calculated in the sweep above
        #: NJA - optimization is done to reduce precision errors 
        if any(sp_sens.poles > 0):
            if nearest_nyquist < sm:
                res = sp.optimize.minimize(nyquist_min, nearest_nyquist_freq, method='SLSQP', options={
                    'finite_diff_rel_step': 1e-8})
                # Make sure this didn't fail
                if res.status != 0:
                    # if optimization failed, just use the closest value from the initial sweep
                    sm2 = nearest_nyquist
                else:
                    sm2 = min(abs(res.fun), abs(nearest_nyquist))

                # Dubious error catching because of strange nyquist shapes in unstable systems
                sm_list = [sm, sm2]
                mag_list = [np.abs(sm_mag), np.abs(mag_at_min)]
                sm = sm_list[np.argmax(mag_list)]
            sm *= -1  # Flip sign to have a "negative sensitivity margin" because it's unstable
        else: # system is stable
            res = sp.optimize.minimize(nyquist_min, nearest_nyquist_freq, method='SLSQP',
                                       options={'finite_diff_rel_step': 1e-6})
            if res.status != 0:
                 # if optimization failed, just use the closest value from the initial sweep
                sm = nearest_nyquist
            else:
                sm = min(res.fun, nearest_nyquist)

    return sm

def interp_plant(linturb, v, return_scipy=True):
    '''
    Interpolate linear turbine plant 

    Inputs
    ------
    linturb: object
        LinearTurbineModel object
    v: float
        wind speed to interpolate linturb at
    return_scipy: bool, optional
        True:  return type is scipy's StateSpaceContinuous
        False: return type is python control toolbox's StateSpace

    Returns
    -------
    P: StateSpaceContinous or StateSpace
    '''

    # Find interpolated plant on v
    if np.shape(linturb.A_ops)[2] > 1:
        Ap = interp_matrix(linturb.u_h, linturb.A_ops, v)
        Bp = interp_matrix(linturb.u_h, linturb.B_ops, v)
        Cp = interp_matrix(linturb.u_h, linturb.C_ops, v)
        Dp = interp_matrix(linturb.u_h, linturb.D_ops, v)
    else: 
        Ap = np.squeeze(linturb.A_ops, axis=2)
        Bp = np.squeeze(linturb.B_ops, axis=2)
        Cp = np.squeeze(linturb.C_ops, axis=2)
        Dp = np.squeeze(linturb.D_ops, axis=2)

    if return_scipy:
        P = sp.signal.StateSpace(Ap, Bp, Cp, Dp)
    else:
        P = co.StateSpace(Ap, Bp, Cp, Dp)

    return P


def interp_pitch_controller(controller, v, return_scipy=True):
    '''
    Interpolate ROSCO toolbox pitch controller and return linear model

    Inputs
    ------
    controller: object
        ROSCO toolbox controller object
    v: float
        wind speed to interpolate linturb at
    return_scipy: bool, optional
        True:  return type is scipy's StateSpaceContinuous
        False: return type is python control toolbox's StateSpace

    Returns
    -------
    P: StateSpaceContinous or StateSpace
    '''

    # interpolate controller on v
    pitch = interp_matrix(controller.v, controller.pitch_op, v)
    kp = float(interp_matrix(controller.pitch_op_pc, controller.pc_gain_schedule.Kp, pitch))
    ki = float(interp_matrix(controller.pitch_op_pc, controller.pc_gain_schedule.Ki, pitch))
    if return_scipy:
        A, B, C, D = sp.signal.tf2ss([kp, ki], [1, 0])
        Cs = sp.signal.StateSpace(A, B, C, D)
    else:
        Cs = co.tf2ss(co.TransferFunction([kp, ki], [1, 0]))

    return Cs


def add_pcomp(linturb, k_float):
    '''
    Incorporate the effect of a parallel compensation term to a linear turbine model.

    Inputs
    ------
    linturb: object
        LinearTurbineModel object
    k_float: float
        parallel compensation feedback gain

    Returns
    -------
    linturb2: object
        Modified linturb with parallel compensation term included
    '''

    # Find relevant state and input indices
    state_str = 'derivative of 1st tower fore-aft'
    state_idx = np.flatnonzero(np.core.defchararray.find(
        linturb.DescStates, state_str) > -1).tolist()
    input_str = 'collective blade-pitch'
    input_idx = np.flatnonzero(np.core.defchararray.find(
        linturb.DescCntrlInpt, input_str) > -1).tolist()

    # Modify linear system to be \dot{x} = (A+B_fK)x + Bu, 
    #   - B_fKx accounts for parallel compensation in the linear system
    K = np.zeros((linturb.B_ops.shape[1], linturb.A_ops.shape[1], linturb.A_ops.shape[2]))
    K[input_idx, state_idx, :] = -k_float # NJA: negative to account for OpenFAST linearization sign conventions

    linturb2 = copy.copy(linturb)
    linturb2.A_ops = linturb.A_ops + linturb.B_ops * K

    return linturb2


def interp_matrix(x, matrix_3D, q):
    q = np.clip(q, x.min(), x.max())
    f_m = sp.interpolate.interp1d(x, matrix_3D)

    return f_m(np.squeeze(q))


def _convert_to_ss(sys):
    if isinstance(sys, (int, float, complex,
                        np.float64, np.float32,
                        np.int64, np.int32,
                        np.complex128, np.complex64)):
        ss = sp.signal.StateSpace([0], [0], [0], sys)
    else:
        ss = sys

    return ss


def feedback(sys1, sys2, sign=-1):
    """
    Feedback interconnection between two LTI systems.

    This underlying methods here are pulled from the python 
    control toolbox, but made to work with scipy state space 
    objects.
    """
    sys1 = _convert_to_ss(sys1)
    sys2 = _convert_to_ss(sys2)

    # Check to make sure the dimensions are OK
    if (sys1.inputs != sys2.outputs) or (sys1.outputs != sys2.inputs):
        raise ValueError("State space systems don't have compatible "
                         "inputs/outputs for feedback.")
    if sys1.dt != sys2.dt:
        raise ValueError(
            "Both state space systems must have the same timestep (dt), or both be continuous time.")
    else:
        dt = sys1.dt

    A1 = sys1.A
    B1 = sys1.B
    C1 = sys1.C
    D1 = sys1.D
    A2 = sys2.A
    B2 = sys2.B
    C2 = sys2.C
    D2 = sys2.D

    F = np.eye(sys1.inputs) - sign * np.dot(D2, D1)
    if np.linalg.matrix_rank(F) != sys1.inputs:
        raise ValueError("I - sign * D2 * D1 is singular to working precision.")

    # Precompute F\D2 and F\C2 (E = inv(F))
    # We can solve two linear systems in one pass, since the
    # coefficients matrix F is the same. Thus, we perform the LU
    # decomposition (cubic runtime complexity) of F only once!
    # The remaining back substitutions are only quadratic in runtime.
    E_D2_C2 = np.linalg.solve(F, np.concatenate((D2, C2), axis=1))
    E_D2 = E_D2_C2[:, :sys2.inputs]
    E_C2 = E_D2_C2[:, sys2.inputs:]

    T1 = np.eye(sys1.outputs) + sign * np.dot(D1, E_D2)
    T2 = np.eye(sys1.inputs) + sign * np.dot(E_D2, D1)

    A = np.concatenate(
        (np.concatenate(
            (A1 + sign * np.dot(np.dot(B1, E_D2), C1),
             sign * np.dot(B1, E_C2)), axis=1),
         np.concatenate(
             (np.dot(B2, np.dot(T1, C1)),
              A2 + sign * np.dot(np.dot(B2, D1), E_C2)), axis=1)),
        axis=0)
    B = np.concatenate((np.dot(B1, T2), np.dot(np.dot(B2, D1), T2)), axis=0)
    C = np.concatenate((np.dot(T1, C1), sign * np.dot(D1, E_C2)), axis=1)
    D = np.dot(D1, T2)

    if dt:
        return sp.signal.StateSpace(A, B, C, D, dt=dt)
    else:
        return sp.signal.StateSpace(A, B, C, D)
