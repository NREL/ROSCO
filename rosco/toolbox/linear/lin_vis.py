'''
Visualization helpers for linear models
'''
from rosco.toolbox.linear.lin_util import add_pcomp, pc_openloop
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp


class lin_plotting():
    def __init__(self, controller, turbine, linturb):
        '''
        Parameters
        ----------
        controller: object
            ROSCO controller object
        turbine: object
            ROSCO turbine object
        linturb: object
            ROSCO linturb object
        '''
        self.turbine    = turbine
        self.controller = controller
        self.linturb    = linturb
        
    def plot_nyquist(self, u, omega, k_float=0.0, xlim=None, ylim=None, fig=None, ax=None, num='Nyquist', **kwargs):
        '''
        Plot nyquist diagram
        
        Parameters:
        -----------
        u: float
            windspeed for linear model
        omega: float
            controller bandwidth
        k_float: float, optional
            parallel compensation feedback gain
        '''
        if fig and ax:
            self.fig = fig
            self.ax = ax
        else:
            self.fig, self.ax = plt.subplots(1,1,num=num)
        w, H = self.get_nyquistdata(u, omega, k_float=k_float)
        self.line, = self.ax.plot(H.real, H.imag,  **kwargs)
        plt.scatter(-1,0,marker='x',color='r')

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        plt.xlabel('Real Axis')
        plt.ylabel('Imaginary Axis')
        plt.title('Nyquist Diagram\n$u = {}, \omega = {}, k_f = {}$'.format(u,omega,k_float))
        plt.grid(True)


    def get_nyquistdata(self, u, omega, k_float=0.0):
        '''
        Gen nyquist diagram data to plot
        
        Parameters:
        -----------
        u: float
            windspeed for linear model
        omega: float
            controller bandwidth
        k_float: float, optional
            parallel compensation feedback gain
        '''
        self.controller.omega_pc = omega
        self.controller.U_pc = u
        self.controller.tune_controller(self.turbine)
        if k_float:
            linturb = add_pcomp(self.linturb, k_float)
        else:
            linturb = self.linturb
        sys_ol = pc_openloop(linturb, self.controller, u)
        sys_sp = sp.signal.StateSpace(sys_ol.A, sys_ol.B, sys_ol.C, sys_ol.D)
        w, H = sp.signal.freqresp(sys_sp)

        return w, H
    
def sm_circle(r):
    theta = np.linspace(0,2*np.pi,100)
    x = r * np.cos(theta) - 1
    y = r * np.sin(theta)
    
    return x,y
