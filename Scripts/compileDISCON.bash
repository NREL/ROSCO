# Cd to correct directory
cd ../Source 
# Remove old files
rm -rf *.o *.mod *.a

# Compile individual codes
gcc -c --free-form -ffree-line-length-0 -fPIC -Wall FunctionToolbox.f90
gcc -c --free-form -ffree-line-length-0 -fPIC -Wall Filters.f90
gcc -c --free-form -ffree-line-length-0 -fPIC -Wall IPC.f90

# COMPILE DISCON
#gcc -c --free-form -ffree-line-length-0 DISCON.f90
rm -f ../DISCON/DISCON.so
#gcc -shared -ffree-line-length-0 -Wall -o ../DISCON/DISCON.so -fPIC DISCON.f90 FunctionToolbox.f90 Filters.f90 IPC.f90
gcc -shared -ffree-line-length-0 -o ../DISCON/DISCON.so -fPIC DISCON.f90 FunctionToolbox.f90 Filters.f90 IPC.f90
rm -rf *.o *.mod

echo 'The output file is: "../DISCON/DISCON.so".' 

# Return to initial directory
cd ../Scripts