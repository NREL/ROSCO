import platform
import os
import sysconfig

if platform.system() == "Windows":
    lib_ext = ".dll"
elif platform.system() == "Darwin":
    lib_ext = ".dylib"
else:
    lib_ext = ".so"

rosco_dir = os.path.dirname( os.path.abspath(__file__) )
libname = "libdiscon"

lib_path = [os.path.join(rosco_dir, "lib", libname+lib_ext), # pip installs (regular and editable)
            os.path.join(os.path.dirname( os.path.dirname( rosco_dir )), "local", "lib", libname+lib_ext), # WEIS library
            os.path.join(os.path.dirname( sysconfig.get_path('stdlib') ), libname+lib_ext), # conda installs
            os.path.join(os.path.dirname( sysconfig.get_path('stdlib') ), "lib", libname+lib_ext), # conda installs
            os.path.join(os.path.dirname( sysconfig.get_path('stdlib') ), "Library", "lib", libname+lib_ext), # conda installs
            os.path.join( sysconfig.get_path('platlib'), "rosco", "lib", libname+lib_ext), # system-wide pip installs
            os.path.join( sysconfig.get_config_var("userbase"), "lib", "python", "site-packages", "rosco", "lib", libname+lib_ext), # system wide local
            ]

discon_lib_path = None
for p in lib_path:
    if os.path.exists(p):
        discon_lib_path = str(p)
        break

if discon_lib_path is None:
    raise Exception(f"Cannot find {libname+lib_ext} in {lib_path}")
        
