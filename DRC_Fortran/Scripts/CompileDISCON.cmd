:: Compiling DISCON

:: Main FAST directory:
:: Change this to the absolute path if this script is not located in the FAST main folder.
:: In that case also make sure the correct drive is selected 
set "FASTdir=."

:: Go to the correct drive
:: C:

:: Remove old .dll file
DEL %FASTdir%\CertTest\5MW_Baseline\ServoData\DISCON_gwin32.dll

:: Compile new .dll file
cd %FASTdir%\Compiling
mingw32-make.exe

pause
