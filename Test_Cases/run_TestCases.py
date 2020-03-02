# ------- Run_TestCases -------
# Run the simulation cases in the Test_Cases folder
# Plot some basic results to make sure they worked
# -----------------------------

# Import and define modules and classes
import os
import matplotlib.pyplot as plt
from ROSCO_toolbox import utilities as ROSCO_Utilities
fast_io = ROSCO_Utilities.FAST_IO()

# Define call for OpenFAST
fastcall = 'openfast_dev'

# Define folder names in Test_Cases to run
testcases = [
                # '5MW_Step',      # Step wind (General Functionality)
                '5MW_Turb_NR',   # Near rated class A turbulence (Setpoint smoothing)
                # '5MW_Turb_NR_ps',   # Near rated class A turbulence (Setpoint smoothing, peak shaving)
                # '5MW_Ramp'      # Ramp from 5-30 m/s in 200 seconds (Shutdown)
            ]

# Define data to plot 
cases = {}
cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed']

# Run test cases
for case in testcases:
    # Run
    OpenFAST_dir = case
    fast_io.run_openfast(OpenFAST_dir,fastcall=fastcall, chdir=True)

# Plot the results
for cid, case in enumerate(testcases):
    # Load outdata
    filename = os.path.join(case,'{}.outb'.format(case))
    print(filename)
    allinfo, alldata = fast_io.load_output(filename)

    # Plot some results
    fast_io.plot_fast_out(cases, allinfo, alldata,showplot=False,fignum=cid)

plt.show()
