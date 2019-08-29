# Example_04
# Load a turbine model and use to tune the controller


from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import sim as wtc_sim

# Initialiize a turbine class
turbine = wtc_turbine.Turbine()

# Load quick from python
turbine.load('saved_turbine.p')