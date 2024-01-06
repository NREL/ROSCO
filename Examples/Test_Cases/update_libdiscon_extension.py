import glob
import platform
import os

if platform.system() == 'Windows':
    sfx = 'dll'
elif platform.system() == 'Darwin':
    sfx = 'dylib'
else:
    sfx = 'so'

if __name__ == "__main__":
    this_dir   = os.path.dirname(os.path.abspath(__file__))
    servo_list = glob.glob(os.path.join(this_dir, '*/*Servo*.dat'))

    for ifile in servo_list:
        # Read in current ServoDyn file
        with open(ifile, "r") as f:
            lines = f.readlines()

        # Write correction
        with open(ifile, "w") as f:
            for line in lines:
                f.write(line.replace('libdiscon.so', f'libdiscon.{sfx}'))
                
