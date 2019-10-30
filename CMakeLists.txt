
cmake_minimum_required(VERSION 3.6)
project(DRC_Fortran VERSION 0.1 LANGUAGES Fortran)

set(CMAKE_Fortran_MODULE_DIRECTORY "${CMAKE_BINARY_DIR}/ftnmods")

# Sets the optimization level to -O2 and includes -g 
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(SOURCES
    src/Constants.f90
    src/ControllerBlocks.f90
    src/Controllers.f90
    src/DISCON.f90
    src/ROSCO_Types.f90
    src/Filters.f90
    src/Functions.f90
    src/ReadSetParameters.f90
)

add_library(discon SHARED ${SOURCES})
