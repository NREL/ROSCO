#================================================================================#
# This makefile created by B. Jonkman on 2-Apr-2013,                             #
# adapted from Crunch (M. Buhl on 25-Jan-2013).                                  #
# (c) 2013 National Renewable Energy Laboratory                                  #
#                                                                                #
# This makefile has been tested on Windows 7 with gfortran.                      #
# This makefile works with mingw32-make.exe.                                     #
#                                                                                #
# It was designed to be used with discon.f90 for the Bladed DLL Interface        #
#================================================================================#

   # 32-bit or 64-bit?
BITS = 32
#BITS = 64


   # Location of source files for the DLL.
   # You may need to change these for your DLL.

DLL_DIR     = .

SOURCE_FILE = DRC_Types.f90 ReadParameters.f90 FunctionToolbox.f90 Filters.f90 IPC.f90 DISCON.f90


   # Name of compiler to use and flags to use.
FC      = gfortran
FFLAGS  = -O2 -m$(BITS) -fbacktrace -ffree-line-length-none -x f95-cpp-input -C
LDFLAGS = -shared -O2 -m$(BITS) -fbacktrace

# other useful gfortran options:
#  -fdefault-real-8 -fcheck=bounds -std=f2003 -O0 -v
# -Wl,--out-implib,DISCON.lib


   # Destination and RootName for executable

OUTPUT_NAME = DISCON
DEST_DIR    = ../DISCON

   #==========================================================#
   # You should not need to change anything beyond this point #
   #==========================================================#

   # System-specific settings.

ifeq ($(OS),Windows_NT)
      # Windows
   DEL_CMD   = del
   EXE_EXT   = _gwin$(BITS).dll
   INTER_DIR = Obj_win$(BITS)
   MD_CMD    = @mkdir
   OBJ_EXT   = .obj
   PATH_SEP  = \\
   SYS_FILE  = SysGnuWin
else

   FFLAGS := $(FFLAGS) -DIMPLICIT_DLLEXPORT

      # Linux
   DEL_CMD   = rm -f
   EXE_EXT   = _glin$(BITS).so
   INTER_DIR = Obj_lin$(BITS)
   MD_CMD    = @mkdir -p
   OBJ_EXT   = .o
   PATH_SEP  = /
   SYS_FILE  = SysGnuLinux

   UNAME := $(shell uname -s)

   ifneq ($(UNAME), Darwin)
      FFLAGS     := $(FFLAGS) -fPIC
      LDFLAGS    := $(LDFLAGS) -fPIC
   endif
endif


   # Source files (by module)

DLL_SOURCES =          \
	$(SOURCE_FILE)

vpath %.f90 $(DLL_DIR)
vpath %.mod $(INTER_DIR)
vpath %.obj $(INTER_DIR)

ALL_OBJS      = $(DLL_SOURCES:.f90=.obj)

   # Rule to do everything.

all:     default
default:  $(INTER_DIR) $(DEST_DIR)/$(OUTPUT_NAME)$(EXE_EXT)

   # General rules for compliling the files.

%.obj: %.f90
	$(FC)  $(FFLAGS) -c $< -o $(INTER_DIR)/$@ -J $(INTER_DIR) -B $(INTER_DIR)


   #  Dependency rules.
#$(SYS_FILE).obj:                   $(PREC).obj


   # Make sure the destination directory for the intermediate files exist.

$(INTER_DIR):
	$(MD_CMD) $(INTER_DIR)


   # For linking DLL.

$(DEST_DIR)/$(OUTPUT_NAME)$(EXE_EXT): $(ALL_OBJS) | $(INTER_DIR)
	$(FC) $(LDFLAGS) -I $(INTER_DIR) -o $(DEST_DIR)/$(OUTPUT_NAME)$(EXE_EXT) \
	$(foreach src, $(ALL_OBJS), $(addprefix $(INTER_DIR)/,$(src)))

   # Cleanup afterwards.

clean:
	$(DEL_CMD) $(INTER_DIR)$(PATH_SEP)*.mod $(INTER_DIR)$(PATH_SEP)*.obj

#gfortran -shared -O2 -m32 -fbacktrace -ffree-line-length-none -x f95-cpp-input -C ../CertTest/5MW_Baseline/ServoData/Source/DISCON.f90