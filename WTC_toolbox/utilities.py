# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import matplotlib.pyplot as plt
import os

class Utilities():
    
    def __init__(self):
        ''' 
        A collection of utilities that may be useful for using the tools made accessbile in this toolbox
        '''

    def run_openfast(self,fast_dir,fastcall='OpenFAST',fastfile=None,):
        '''
        Runs a openfast openfast simulation 
            ! NJA - should include a processing script to return output data to a variable 
            !       Might only work on unix machines? Not sure how OpenFAST is generally run on Windows. 
        Parameters:
        ------------
            fast_dir: string
                    Name of OpenFAST directory containing input files.
            fastcall: string, optional
                    Line used to call openfast when executing from the terminal.
            fastfile: string, optional
                    Filename for *.fst input file. Function will find *.fst if not provided.
        '''

        # Define OpenFAST input filename
        if fastfile:
            print('Using {} to run OpenFAST simulation'.format(fastfile))
        else:
            for file in os.listdir(fast_dir):
                if file.endswith('.fst'):
                    fastfile = file
                    print(file)

        # save starting file path -- note: This is an artifact of needing to call OpenFAST from the same directory as DISCON.IN
        original_path = os.getcwd()
        # change path, run sim
        os.chdir(fast_dir)
        os.system('{} {}'.format(fastcall, os.path.join(fast_dir,'*.fst')))
        # return to original path
        os.chdir(original_path)

    def plotFASTout(self):
        '''
        Plot OpenFAST outputs 
            - NJA: this is a good place to emulate Post_LoadFastOut.m
        '''

    def readFASTout(self):
        '''
        Read OpenFAST output files. Might want to leverage AeroelasticSE here.
        '''

