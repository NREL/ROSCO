#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import os

# Wind file for sheared 18 m/s wind with 30 degree direction.
# Time	Wind	Wind	Vert.	Horiz.	Vert.	LinV	Gust
#	    Speed	Dir	    Speed	Shear	Shear	Shear	Speed

class HH_WindFile(object):
    '''
    Hub height wind file


    __init__: setup file based on wind_type, t_max, dt
    write:    write file
    
    '''

    def __init__(self,**kwargs):


        self.wind_type  = 'step'   # wind file type: 'step' supported
        self.dt         = 0.05    # wind time step
        self.filename   = self.wind_type + '.wnd'  # Path to fst directory files
        self.T_max      = 600

        # set default wind input (step)
        self.time           = [0,300,300+self.dt,600]
        self.wind_speed     = [10,10,14,14]
        self.wind_dir       = [0] * len(self.time)
        self.vert_speed     = [0] * len(self.time)
        self.horiz_shear    = [0] * len(self.time)
        self.vert_shear     = [0] * len(self.time)
        self.linv_shear     = [0] * len(self.time)
        self.gust_speed     = [0] * len(self.time)

        # Optional population class attributes from key word arguments
        for k, w in kwargs.items():
            try:
                setattr(self, k, w)
            except:
                pass

        super(HH_WindFile, self).__init__()


    def write(self):
        if not os.path.isdir(os.path.dirname(self.filename)):
            os.makedirs(os.path.dirname(self.filename))
        with open(self.filename,'w') as f:
            f.write('!\tTime\tWind Speed\tWind Dir\tVert. Spd.\tHoriz. Shr.\t Vert. Shr.\t LinV. Shr.\tGust Speed\n')
            for t, ws, wd, vs, hs, vs, ls, gs in zip(self.time,self.wind_speed,self.wind_dir,self.vert_speed, \
                self.horiz_shear, self.vert_shear, self.linv_shear, self.gust_speed):
                f.write('{:6.6f}\t{:6.6f}\t{:6.6f}\t{:6.6f}\t{:6.6f}\t{:6.6f}\t{:6.6f}\t{:6.6f}\n'.format(
                    t,ws,wd,vs,hs,vs,ls,gs))


    def plot(self):
        plt.plot(self.time,self.wind_speed)
        plt.show()

class HH_StepFile(HH_WindFile):
    
    def __init__(self):

        self.u_start    = 10
        self.u_end      = 14
        self.t_step     = 300
        self.t_max      = 600
        self.dt         = 0.05

        self.wind_type          = 'step'   # wind file type: 'step' supported
        self.wind_directory     = '.'

        self.update()
       

    def update(self):
        self.filename   = os.path.join(self.wind_directory,'{}_{:.1f}_{:.1f}.wnd'.format(self.wind_type,self.u_start,self.u_end))

        # set default wind input (step)
        self.time           = [0,self.t_step,self.t_step+self.dt,self.t_max]
        self.wind_speed     = [self.u_start,self.u_start,self.u_end,self.u_end]
        self.wind_dir       = [0] * len(self.time)
        self.vert_speed     = [0] * len(self.time)
        self.horiz_shear    = [0] * len(self.time)
        self.vert_shear     = [0] * len(self.time)
        self.linv_shear     = [0] * len(self.time)
        self.gust_speed     = [0] * len(self.time)

# def gen_step_wind(,u_start,u_end,t_step,t_max,dt=0.05)
#     '''
#         Generate wind input with single step in wind
#         No shear, direction or anything else

#         Inputs:
#             - filename: where to write file
#             - u_start: starting wind speed
#             - u_end: ending wind speed
#             - t_step: time of wind step

#     '''

#     # time breakpoints
#     t_bp = [0,t_step,t_step+dt,t_max]
#     u_bp = [u_start,u_start,u_end,u_end]


#     Uinf = 8.0
#     max_wdir_delta = 30.0
#     T = 600.0
#     dt = 0.5

#     t = np.arange(0,T+dt,dt)
#     wdir = max_wdir_delta * np.sin(2*np.pi*t/T)
#     for ti,wdi in zip(t,wdir):
#         print('{:>6.1f}\t{:.1f}\t{:.1f}\t0.0\t0.0\t0.0\t0.0\t0.0'.format(ti,Uinf,wdi))



if __name__ == "__main__":
    hh_step = HH_StepFile()

    hh_step.t_step  = 400
    hh_step.t_max   = 800
    hh_step.u_start = 10
    hh_step.u_end   = 11
    hh_step.update()

    hh_step.write()
    
    print('here')
