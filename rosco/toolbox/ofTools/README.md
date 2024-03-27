# OpenFAST Tools
A set of OpenFAST tools is provided to ease the use of the ROSCO toolbox in conjuction with [OpenFAST](https://github.com/openfast/openfast).

OpenFAST can be installed using conda-forge using
```
conda install -c conda-forge openfast
```

Note that most of these scripts are copy and pasted from [weis's aeroleasticse](https://github.com/WISDEM/WEIS/tree/master/weis/aeroelasticse)

### case_gen
The scripts found in the case_gen folder can be used to run large sets of openfast simulations. This can be done ad-hoc, or for specific DLCs. 

### fast_io
The scripts in the fast_io folder are generally used to read or write OpenFAST input files. Note that all files starting with "FAST_*" should generally be the same as those found in aeroelasticse (amongst a few other scripts). 
- output_processing.py can be used to read OpenFAST output binary or text files, and then plot the output time series or power spectra.
- read_fast_input.py can be used to generically read OpenFAST input decks in a somewhat more flexible way than that in FAST_reader.py, so may be useful if the user's OpenFAST input files are out of date. 