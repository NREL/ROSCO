cmake_minimum_required(VERSION 3.6)
project(ROSCO VERSION 2.6.0 LANGUAGES Fortran C)

set(CMAKE_Fortran_MODULE_DIRECTORY "${CMAKE_BINARY_DIR}/ftnmods")

if (NOT CMAKE_BUILD_TYPE)
  # Sets the optimization level to -O2 and includes -g for debugging
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

message(STATUS "CMAKE_Fortran_COMPILER_ID = ${CMAKE_Fortran_COMPILER_ID}")
if(APPLE OR UNIX)
# Enable .dll export
if (CMAKE_Fortran_COMPILER_ID STREQUAL "Intel")
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -r8 -double-size 64 -cpp -no-wrap-margin")
else()
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -ffree-line-length-0 -fdefault-real-8 -fdefault-double-8 -cpp")  
endif()
elseif (WIN32)
if (CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
# Ensure static linking to avoid requiring Fortran runtime dependencies
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -ffree-line-length-0 -static-libgcc -static-libgfortran -static -fdefault-real-8 -fdefault-double-8 -cpp")
elseif (CMAKE_Fortran_COMPILER_ID STREQUAL "Intel")
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -libs:static -free -static -fpp -real-size:64 -double-size:64")
#     set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} /ffree-line-length-0 /static-libgcc /static-libgfortran /static /fdefault-real-8 /fdefault-double-8 /cpp")
endif()
endif()

# Enable ZMQ_Client if compiler flag is set
option(ZMQ_CLIENT "Enable use of ZeroMQ client" off)
if(ZMQ_Client)
  set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -lzmq")
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
    src/ROSCO_Helpers.f90
    src/ROSCO_IO.f90
    src/ZeroMQInterface.f90
    src/ExtControl.f90
)

if (${CMAKE_Fortran_COMPILER_ID} STREQUAL "GNU")
  if (WIN32)
    set(NWTC_SYS_FILE src/SysFiles/SysGnuWin.f90)
  elseif (APPLE OR UNIX OR CYGWIN)
    set(NWTC_SYS_FILE src/SysFiles/SysGnuLinux.f90)
  endif ()
elseif (${CMAKE_Fortran_COMPILER_ID} MATCHES "^Intel")
  if (APPLE OR UNIX)
    set(NWTC_SYS_FILE src/SysFiles/SysIFL.f90)
  elseif (WIN32)
    set(NWTC_SYS_FILE src/SysFiles/SysIVF.f90)
  endif (APPLE OR UNIX)
endif ()

if (NWTC_SYS_FILE)
  message("-- Setting system file as: ${NWTC_SYS_FILE}")
  list(APPEND SOURCES ${NWTC_SYS_FILE})
else (NWTC_SYS_FILE)
  message(FATAL_ERROR "Cannot determine system file used with NWTC_Library")
endif (NWTC_SYS_FILE)

# Library
if(ZMQ_CLIENT)
	# Find ZeroMQ installation
	find_package(PkgConfig)
	pkg_check_modules(PC_ZeroMQ libzmq)
	find_path(ZeroMQ_INCLUDE_DIR
			NAMES zmq.h
			PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
			)
	find_library(ZeroMQ_LIBRARY
			NAMES zmq
			PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
			)
	include_directories(${ZeroMQ_INCLUDE_DIR})


	# Compile C-based ZeroMQ client as object library
	add_compile_options(-I${ZeroMQ_INCLUDE_DIR} -l${ZeroMQ_LIBRARY} -fPIC)
	add_library(zmq_client OBJECT ../src/zmq_client.c)

	# Add definition
	add_definitions(-DZMQ_CLIENT="TRUE")
    set(SOURCES ${SOURCES}
	$<TARGET_OBJECTS:zmq_client>)
endif()

add_library(discon SHARED ${SOURCES})

if(ZMQ_CLIENT)
	target_include_directories(discon PUBLIC ${ZeroMQ_INCLUDE_DIR})
	target_link_libraries(discon PUBLIC ${ZeroMQ_LIBRARY})
endif()


install(TARGETS discon
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")
endif()
