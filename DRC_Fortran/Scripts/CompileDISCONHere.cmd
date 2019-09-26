:: Compiling DISCON

:: Main DISCON source-file directory:
set "DISCONSourceDir=..\"

:: Remove old .dll file
DEL %DISCONSourceDir%\DISCON\DISCON_gwin64.dll

:: Compile new .dll file
del /f /s /q Obj_win64 1>nul
rmdir /s /q Obj_win64

cd %DISCONSourceDir%\Source
::mingw32-make.exe
make.exe

del /f /s /q Obj_win64 1>nul
rmdir /s /q Obj_win64

pause
