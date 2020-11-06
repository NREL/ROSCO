
'''
Run ROSCO lite testing scripts for controller functionality verification
'''
import os
import glob
import ROSCO_testing
import importlib

from weis.aeroelasticse.Util import FileTools
from pCrunch import pdTools
from pCrunch import Processing, Analysis


def run_testing(turbine2test, testtype, rosco_binaries=[], discon_files=[], **kwargs):
    '''
    Run and post-process controller testing scripts. 

    NOTE: There are 

    Parameters:
    -----------
    turbine2test: str
        Reference turbine to test: IEA-15MW or NREL-5MW
    testtype: str
        Type of test: 'lite' - small set of 5 minute simulations
                      'heavy' - large set of 10 minute simulation (DLC 1.3 & 1.4 subset)
                      'binary-comp' - compare binaries
                      'discon-comp' - compare discon files
    rosco_binaries: list-like
        List of strings with paths to ROSCO binaries. Len>2 needed for binary-comp!
        NOTE: Will use the first binary if testtype='lite' or 'heavy'. s
    discon_files: list-like, only for discon-comp
        List of strings with paths to DISCON.IN files - Neede for discon-comp!
    ''' 
    ## =================== INITIALIZATION ===================
    # Instantiate  class
    rt = ROSCO_testing.ROSCO_testing(**kwargs)

    # Post Processing Parameters
    reCrunch = True                     # re-run pCrunch?

    #### ============================================ ####
    #                     THE ACTION                     #
    # ----- Shouldn't need to change anything here ----- #

    # Check validity inputs
    if testtype.lower() == 'binary-comp' and len(rosco_binaries) < 2:
        raise ValueError('You need at least two binaries to compare for testtype = "binary-comp"!')
    if testtype.lower() == 'discon-comp' and len(discon_files) < 2:
        raise ValueError('You need at least two discon files to compare for testtype = "binary-comp"!')

    # Setup test turbine
    if turbine2test == 'NREL-5MW':
        rt.Turbine_Class = 'I'
        rt.Turbulence_Class = 'A'
        rt.FAST_directory = os.path.join(os.getcwd(), '../Test_Cases/NREL-5MW')
        rt.FAST_InputFile = 'NREL-5MW.fst'
    elif turbine2test == 'IEA-15MW':
        rt.Turbine_Class = 'I'
        rt.Turbulence_Class = 'B'
        rt.FAST_directory = os.path.join(os.getcwd(), '../Test_Cases/IEA-15-240-RWT-UMaineSemi')
        rt.FAST_InputFile = 'IEA-15-240-RWT-UMaineSemi.fst'
    else:
        raise ValueError('{} is not an available turbine to test!'.format(turbine2test))

    # Run test
    if testtype.lower() == 'lite':
        rt.rosco_path = rosco_binaries[0]
        rt.ROSCO_Test_lite()
    elif testtype.lower() == 'heavy':
        rt.rosco_path = rosco_binaries[0]
        rt.ROSCO_Test_heavy()
    elif testtype.lower() == 'binary-comp':
        rt.ROSCO_Controller_Comp(rosco_binaries)
    elif testtype.lower() == 'discon-comp':
        rt.ROSCO_DISCON_Comp(discon_files)
    else:
        raise ValueError('{} is an invalid test type!'.format(testtype))



    # ----------- Post process ----------

    # Set up pCrunch
    # Initialize processing classes
    fp = Processing.FAST_Processing()

    if testtype.lower() in ['lite', 'heavy']:
        # Load generated .yaml for outfile names
        case_info = FileTools.load_yaml(os.path.join(rt.runDir,'case_matrix.yaml'), package=1)
        casenames = case_info['Case_Name']
        if rt.outfile_fmt >= 2:
            outFileNames = [os.path.join(rt.runDir,fname+'.outb') for fname in casenames]
        else:
            outFileNames = [os.path.join(rt.runDir,fname+'.out') for fname in casenames]

        # Set some processing parameters
        fp.OpenFAST_outfile_list = outFileNames
        fp.t0 = 100
        if rt.cores > 1:
            fp.parallel_analysis = True
        fp.results_dir = os.path.join(rt.runDir, 'stats')
        fp.verbose = True
        fp.save_LoadRanking = True
        fp.save_SummaryStats = True

        # Define load ranking stats
        fp.ranking_vars = [['RotSpeed'],
                            ['RootMyb1', 'RootMyb2', 'RootMyb3'],
                            ['TwrBsMyt']]
        fp.ranking_stats = ['max',
                            'max',
                            'max']
        if turbine2test == 'IEA-15MW':
            fp.ranking_vars += [['PtfmPitch']]
            fp.ranking_stats += ['max']

        # Load and save statistics and load rankings
        if not os.path.exists(os.path.join(fp.results_dir, 'dataset1_LoadRanking.yaml')) or reCrunch:
            stats, load_rankings = fp.batch_processing()

        # Print a summary
        print('----------------------------------------')
        print('\t\t{}'.format(turbine2test))
        print('----------------------------------------')
        print('\t\t{}'.format('Test Outputs'))
        for meas in load_rankings[0].keys():
            try:
                print('Max {} \t{:1.3e}'.format(meas, load_rankings[0][meas]['max'][0]))
            except:
                print('{} is not in load rankings'.format(meas))
        print('----------------------------------------')

    else:
        # Save each set of outputs in a list
        allstats = []
        allrankings = []
        for i in range(max(len(discon_files), len(rosco_binaries))):
            # Define output file folder
            outputDir = os.path.join(os.getcwd(), rt.runDir,'controller_{}'.format(i)) # NJA - this is dependent on the hard-coded folder names in ROSCO_testing.py

            # Load output file paths using case matrices
            case_info = FileTools.load_yaml(os.path.join(outputDir, 'case_matrix.yaml'), package=1)
            casenames = case_info['Case_Name']
            if rt.outfile_fmt >= 2:
                outFileNames = [os.path.join(outputDir, fname+'.outb') for fname in casenames]
            else:
                outFileNames = [os.path.join(outputDir, fname+'.out') for fname in casenames]

            # Set some processing parameters
            fp.OpenFAST_outfile_list = outFileNames
            fp.t0 = 100
            if rt.cores > 0:
                fp.parallel_analysis = True
            fp.results_dir = os.path.join(outputDir, 'stats')
            fp.verbose = True
            fp.save_LoadRanking = True
            fp.save_SummaryStats = True

            # Define load ranking stats
            fp.ranking_vars = [['RotSpeed'],
                            ['RootMyb1', 'RootMyb2', 'RootMyb3'],
                            ['TwrBsMyt']]
            fp.ranking_stats = ['max',
                                'max',
                                'max']
            if turbine2test == 'IEA-15MW':
                fp.ranking_vars += [['PtfmPitch']]
                fp.ranking_stats += ['max']

            # Load and save statistics and load rankings
            if not os.path.exists(os.path.join(fp.results_dir, 'dataset1_LoadRanking.yaml')) or reCrunch:
                stats, load_rankings = fp.batch_processing()
            
            # Save all outputs
            allstats.append(stats)
            allrankings.append(load_rankings[0])

        # Print a summary
        print('----------------------------------------')
        print('\t\t{}'.format(turbine2test))
        print('----------------------------------------')
        print('\t\t{}\t{}'.format('Controller1', 'Controller2'))
        for meas in allrankings[0].keys():
            try:
                print('Max {} \t{:1.3e}\t{:1.3e}'.format(
                    meas, allrankings[0][meas]['max'][0], allrankings[1][meas]['max'][0]))
            except:
                print('{} is not in load rankings'.format(meas))
        print('----------------------------------------')
        
if __name__ == "__main__":

    # WEIS directory, for running openfast, etc.
    weis_dir = os.environ.get('weis_dir')       # works if we do  `export weis_dir=$(pwd)` in WEIS directory in terminal
    weis_dir = '/Users/dzalkind/Tools/WEIS'

    # Setup ROSCO testing parameters
    rt_kwargs = {} 
    rt_kwargs['runDir']     = 'results/lite_testing'        # directory for FAST simulations
    rt_kwargs['namebase']   = 'ROSCO_lite_test'     # Base name for FAST files 
    rt_kwargs['FAST_exe']   = os.path.join(weis_dir,'local','bin','openfast')       # OpenFAST executable path
    rt_kwargs['Turbsim_exe']= os.path.join(weis_dir,'local','bin','turbsim')        # Turbsim executable path
    rt_kwargs['FAST_ver']   = 'OpenFAST'            # FAST version
    rt_kwargs['dev_branch'] = True                  # dev branch of Openfast?
    rt_kwargs['debug_level']= 2                     # debug level. 0 - no outputs, 1 - minimal outputs, 2 - all outputs
    rt_kwargs['overwrite']  = False                 # overwite fast sims?
    rt_kwargs['cores']      = 1                     # number of cores if multiprocessing
    rt_kwargs['mpi_run']    = False                 # run using mpi
    rt_kwargs['mpi_comm_map_down'] = []             # core mapping for MPI
    rt_kwargs['outfile_fmt'] = 2                    # 1 = .txt, 2 = binary, 3 = both

    # ---- Define test type ----
    turbine2test = 'IEA-15MW'   # IEA-15MW or NREL-5MW
    testtype     = 'lite'       # lite, heavy, binary-comp, discon-comp

    # Only fill one of these if comparing controllers
    rosco_binaries = [os.path.join(os.getcwd(), glob.glob('../ROSCO/build/libdiscon*')[0]),
                    os.path.join(os.getcwd(), glob.glob('../ROSCO/build/libdiscon*')[0])] # Differently named libdiscons to compare
    discon_files = []   # Differently named DISCON.IN files to compare


    # Run testing
    run_testing(turbine2test, testtype, rosco_binaries=rosco_binaries, discon_files=discon_files, **rt_kwargs)
