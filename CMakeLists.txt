cmake_minimum_required(VERSION 3.6)
project(ROSCO VERSION 2.0.1 LANGUAGES Fortran)

set(CMAKE_Fortran_MODULE_DIRECTORY "${CMAKE_BINARY_DIR}/ftnmods")

if (NOT CMAKE_BUILD_TYPE)
  # Sets the optimization level to -O2 and includes -g for debugging
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

# Enable .dll export
if(APPLE OR UNIX)
  if (CMAKE_Fortran_COMPILER_ID STREQUAL "Intel")
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT ")
  else()
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -ffree-line-length-0")
  endif()
endif()

set(SOURCES
    src/Constants.f90
    src/ControllerBlocks.f90
    src/Controllers.f90
    src/DISCON.F90
    src/ROSCO_Types.f90
    src/Filters.f90
    src/Functions.f90
    src/ReadSetParameters.f90
)

add_library(discon SHARED ${SOURCES})

install(TARGETS discon
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
