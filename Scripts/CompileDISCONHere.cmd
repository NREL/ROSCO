:: Compiling DISCON

:: Main DISCON source-file directory:
set "DISCONSourceDir=..\"

:: Remove old .dll file
DEL %DISCONSourceDir%\DISCON\DISCON_gwin32.dll

:: Compile new .dll file
del /f /s /q Obj_win32 1>nul
rmdir /s /q Obj_win32

cd %DISCONSourceDir%\Source
mingw32-make.exe

del /f /s /q Obj_win32 1>nul
rmdir /s /q Obj_win32

pause
