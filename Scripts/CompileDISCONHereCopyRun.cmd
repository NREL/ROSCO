:: Compiling DISCON

:: Main DISCON source-file directory:
set "DISCONSourceDir=.."
set "FASTDir=D:\sebastiaanmuld\Dropbox\JW_Sebastiaan\10MWRWT\Sim_YawOffsetIPCFiles\1_DTU_10MW_10ms_5deg_1"
set "ServoDynDir=D:\sebastiaanmuld\Dropbox\JW_Sebastiaan\10MWRWT\Sim_YawOffsetIPCFiles\ServoData"

:: Remove old .dll file
DEL %DISCONSourceDir%\DISCON\DISCON_gwin64.dll
DEL %ServoDynDir%\DISCON.dll

:: Compile new .dll file
del /f /s /q Obj_win64 1>nul
rmdir /s /q Obj_win64

cd %DISCONSourceDir%\Source
::mingw32-make.exe
make.exe

del /f /s /q Obj_win64 1>nul
rmdir /s /q Obj_win64

copy %DISCONSourceDir%\DISCON\DISCON_gwin64.dll %ServoDynDir%\DISCON.dll

openfast_x64 %FASTDir%\DTU_10MW.fst

pause
