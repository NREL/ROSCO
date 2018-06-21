:: Compiling and Testing Discon.f90

echo off

:: Set this to the absolute path of FAST. In that
:: case also make sure the correct drive is selected
set "FASTdir=C:\FAST"

:: Set these to the absolute path of your archive directory. Does not necessarily have
:: to be the current directory
set "ArchiveDir=C:\FASTArchive"

:: Go to the drive where FAST is located
C:

Remove old .dll file
DEL %FASTdir%\CertTest\5MW_Baseline\ServoData\DISCON_gwin32.dll

:: Compile new .dll file
cd %FASTdir%\Compiling
mingw32-make.exe

:: Run Test18
echo off
cd ..\CertTest
echo on
FAST_Win32.exe Test18.fst

echo off
:: Make new directory with correct timestamp

Get the timestamp
for /f "tokens=2 delims==" %%a in ('wmic OS Get localdatetime /value') do set "dt=%%a"
set "YYYY=%dt:~0,4%" & set "MM=%dt:~4,2%" & set "DD=%dt:~6,2%"
set "HH=%dt:~8,2%" & set "Min=%dt:~10,2%"
set "stamp=%YYYY%_%MM%_%DD%_%HH%%Min%"

:: Put together the directory name and create it
set "dbFolder=%ArchiveDir%\%stamp%"
mkdir %dbFolder%

:: Copy debug files to this directory
copy Test18.sum %dbFolder%\Test18.sum
copy Test18.AD.sum %dbFolder%\Test18.AD.sum
copy Test18.ED.sum %dbFolder%\Test18.ED.sum
copy Test18.out %dbFolder%\Test18.out
copy Test18.SrvD.dbg %dbFolder%\Test18.SrvD.dbg
copy Test18.SrvD.dbg2 %dbFolder%\Test18.SrvD.dbg2


:: Echo the debug directory so matlab can potentially pick it up
echo %dbFolder%
pause