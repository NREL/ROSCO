# Cd to correct directory
cd ../Source 

# Set options to 64 bits
sed -i '13s/.*/#BITS = 32/' makefile
sed -i '14s/.*/BITS = 64/' makefile

# Build code
rm -f ../DISCON/DISCON_glin64.so
make clean 
make all
echo 'The output file is: "../DISCON/DISCON_glin64.so".' 

# Return to initial directory
cd ../Scripts