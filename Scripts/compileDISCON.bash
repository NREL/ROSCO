# Cd to correct directory
cd ../Source 
# Remove old files
rm -rf *.o *.mod *.a

# Compile individual codes
gcc -c --free-form -ffree-line-length-0 FunctionToolbox.f90
gcc -c --free-form -ffree-line-length-0 Filters.f90
gcc -c --free-form -ffree-line-length-0 IPC.f90

# COMPILE DISCON
#gcc -c --free-form -ffree-line-length-0 DISCON.f90
rm -f ../DISCON/DISCON.so
#ar rcs ../DISCON/DISCON.a DISCON.o
gcc -shared -ffree-line-length-0 -o ../DISCON/DISCON.so -fPIC DISCON.f90
rm -rf *.o *.mod

echo 'The output file is: "../DISCON/DISCON.so".' 

# Return to initial directory
cd ../Scripts