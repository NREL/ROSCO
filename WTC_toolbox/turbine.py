# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import numpy as numpy
from ccblade import CCAirfoil, CCBlade

class Turbine():
    """
    Class turbine defines a turbine in terms of what is needed to design the controller
    and to run the 'tiny' simulation
    """

    def __init__(self):
        """
        Maybe just initialize the internal variables
        This also lists what will need to be defined
        """
        # Should names match fast or can be more simple
        self.gb_ratio = None # Initialize all to none?

    def load_from_fast(self, fast_folder):
        """
        Load the parameter files directly from a FAST input deck
        """

    def load_from_sowfa(self, fast_folder):
        """
        Load the parameter files directly from a SOWFA directory
        """

    def load_from_csv(self, fast_folder):
        """
        Load from a simple CSV file containing the parameters
        """