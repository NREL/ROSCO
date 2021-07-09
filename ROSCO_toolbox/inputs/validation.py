import os
from wisdem.inputs import load_yaml, write_yaml, validate_without_defaults, validate_with_defaults, simple_types

schema_dir = os.path.dirname(os.path.abspath(__file__))

def load_rosco_yaml(finput):
    rosco_schema = os.path.join(schema_dir,'toolbox_schema.yaml')
    return validate_with_defaults(finput, rosco_schema)


if __name__=='__main__':
    fname = '/Users/dzalkind/Tools/ROSCO/Tune_Cases/NREL5MW.yaml'
    new_input = load_rosco_yaml(fname)
    
    print('here')

