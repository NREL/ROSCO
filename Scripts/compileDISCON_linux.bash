# Make a DISCON folder in root, cd to correct directory
mkdir ../DISCON
cd ../Source 

# Set options to 64 bits
sed -i '13s/.*/#BITS = 32/' makefile
sed -i '14s/.*/BITS = 64/' makefile

# Build code
rm -f ../DISCON/DISCON_glin64.so
rm -f ../DISCON_x64_DRC.so
make clean 
make all
mv ../DISCON/DISCON_glin64.so ../DISCON_x64_DRC.so
echo 'The output file is: "../DISCON_x64_DRC.so".' 

# Return to initial directory
cd ../Scripts
