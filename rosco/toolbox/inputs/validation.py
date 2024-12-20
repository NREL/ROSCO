import os
from wisdem.inputs.validation import _validate

schema_dir = os.path.dirname(os.path.abspath(__file__))

def load_rosco_yaml(finput, rank_0=False):
    rosco_schema = os.path.join(schema_dir,'toolbox_schema.yaml')
    return _validate(finput, rosco_schema, defaults=True, rank_0=rank_0)


if __name__=='__main__':
    fname = '/Users/dzalkind/Tools/ROSCO/Tune_Cases/NREL5MW.yaml'
    new_input = load_rosco_yaml(fname)
    
    print('here')

