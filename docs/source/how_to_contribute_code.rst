.. how_to_contribute_code:

How to contribute code to ROSCO
===============================

ROSCO is an open-source tool, thus we welcome users to submit additions or fixes to the code to make it better for everybody.
When adding new features to ROSCO, we strive to make them as generic as possible, so they can be applied to a variety of turbines.
We also hope that new features are also representative of methods in most OEM turbines.

Issues
------
If you have an issue with ROSCO, a bug to report, or a feature to request, please submit an issue on the GitHub repository.
This lets other users know about the issue.
If you are comfortable fixing the issue, please do so and submit a pull request.

Documentation
-------------
When you add or modify code, make sure to provide relevant documentation that explains the new code.
This should be done in code via comments and also in the Sphinx documentation if you add a new feature or capability.
Look at the .rst files in the :code:`docs` section of the repo or click on :code:`view source` on any of the doc pages to see some examples.

The best place to add documentation for your new feature is in an example of the new feature. 
We are planning to incorporate docstrings from the examples into these docs.

Testing
-------
ROSCO tests its various features through the Examples.  
Ideally, each new feature would have an associated example. 
Most examples are set up to simply run the example and check for simulation failures.
Some good examples check that the functionality is occurring properly.

An automated testing procedure occurs through GitHub Actions; you can see the progress on the GitHub repo under the :code:`Actions` tab, `located here <https://github.com/NREL/ROSCO/actions>`_.
If any example fails, this information is passed on to GitHub and a red X will be shown next to the commit.
Otherwise, if all tests pass, a green check mark appears to signify the code changes are valid.

The examples that are covered are shown in :code:`ROSCO/rosco/test/test_examples.py`.
If you add an example to ROSCO, make sure to add a call to it in the :code:`run_examples.py` script as well.


Pull requests
-------------
Once you have added or modified code, submit a pull request via the GitHub interface.
This will automatically go through all of the tests in the repo to make sure everything is functioning properly.
The main developers of ROSCO will then merge in the request or provide feedback on how to improve the contribution.

