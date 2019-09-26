:: Compiling DISCON dll and run Test 18
:: Pauses afterwards so command window output can be read

:: Change these to the absolute path of FAST. In that
:: case also make sure the correct drive is selected
set "FASTdir=C:\FAST"

:: Go to the correct drive
C:

:: Remove old .dll file
DEL %FASTdir%\CertTest\5MW_Baseline\ServoData\DISCON_gwin32.dll

:: Compile new .dll file
cd %FASTdir%\Compiling
mingw32-make.exe

:: Run Test18
cd ..\CertTest
FAST_Win32.exe Test18.fst

pause
