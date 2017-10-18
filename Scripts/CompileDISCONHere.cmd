:: Compiling DISCON

:: Main DISCON source-file directory:
set "DISCONSourceDir=..\"

:: Remove old .dll file
DEL %DISCONSourceDir%\DISCON\DISCON_gwin32.dll

:: Compile new .dll file
cd %DISCONSourceDir%\Source
mingw32-make.exe

pause
