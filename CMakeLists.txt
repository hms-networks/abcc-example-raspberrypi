#-----------------------------------------------------------------------------
# Requires CMake 3.11 or later (source-less add_executable() ).
#-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.11)

#-----------------------------------------------------------------------------
# Default build type is 'Release' if none is specified.
# Override with
# cmake .. -DCMAKE_BUILD_TYPE=<type> (e.g., Debug).
#
# Note: values passed via -D are cached and persist across reconfigure.
#
# Return to default: 
# cmake .. -U CMAKE_BUILD_TYPE  (or delete build dir).
#-----------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified")
  set(CMAKE_BUILD_TYPE Release)
endif()

#-----------------------------------------------------------------------------
# Creating a user host application project in C.
#-----------------------------------------------------------------------------
project(raspberry_pi_example_project C)

#-----------------------------------------------------------------------------
# Enforce C99 standard (codebase uses C99 features; 
# fail-fast if compiler lacks support)
#-----------------------------------------------------------------------------
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

#-----------------------------------------------------------------------------
# Sanity check that this is run in a supported environment.
#-----------------------------------------------------------------------------
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
   message(FATAL_ERROR
      "Building raspberry_pi_example_project for target system "
      "'${CMAKE_SYSTEM_NAME}' is not supported. "
      "A Linux environment (e.g. Raspberry Pi OS) is required.")
endif()

#-----------------------------------------------------------------------------
# Find the required library.
#-----------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiod)

#-----------------------------------------------------------------------------
# Creating a user host application executable target.
#-----------------------------------------------------------------------------
add_executable(raspberry_pi_example_project)

#-----------------------------------------------------------------------------
# Source (.c) files to add to the executable target. These are 'user unique'.
# Some are related to the CompactCom Driver API.
#-----------------------------------------------------------------------------
set(raspberry_pi_example_SRCS
   ${PROJECT_SOURCE_DIR}/src/main.c
   ${PROJECT_SOURCE_DIR}/src/example_application/abcc_network_data_parameters.c
   ${PROJECT_SOURCE_DIR}/src/example_application/implemented_callback_functions.c
   ${PROJECT_SOURCE_DIR}/src/example_application/Logprint/logprint.c
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_hardware_abstraction.c
)

#-----------------------------------------------------------------------------
# Include (.h) files related to the user host application.
#-----------------------------------------------------------------------------
set(raspberry_pi_example_INCS
   ${PROJECT_SOURCE_DIR}/src/example_application/Logprint/logprint.h
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_driver_config.h
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_software_port.h
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_types.h
)

#-----------------------------------------------------------------------------
# Adding the 'user unique' sources to the user host application executable
# target. (The include files are added to keep the file and directory tree
# structure.)
#-----------------------------------------------------------------------------
target_sources(raspberry_pi_example_project PRIVATE
   ${raspberry_pi_example_SRCS}
   ${raspberry_pi_example_INCS}
)

#-----------------------------------------------------------------------------
# GNU extensions enabled: strerrordesc_np() requires _GNU_SOURCE (glibc only).
#-----------------------------------------------------------------------------
target_compile_definitions(raspberry_pi_example_project PRIVATE _GNU_SOURCE)

#-----------------------------------------------------------------------------
# Directories containing 'user unique' include (.h) files related to the
# CompactCom Driver API.
#-----------------------------------------------------------------------------
set(ABCC_API_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/
  ${PROJECT_SOURCE_DIR}/src/example_application/
  ${PROJECT_SOURCE_DIR}/src/example_application/Logprint/
)

#-----------------------------------------------------------------------------
# The directory containing the Anybus CompactCom Driver API repository.
#-----------------------------------------------------------------------------
set(ABCC_API_DIR ${PROJECT_SOURCE_DIR}/lib/abcc-driver-api)


#-----------------------------------------------------------------------------
# Including the Anybus CompactCom Driver API's CMake module file.
#-----------------------------------------------------------------------------
include(${ABCC_API_DIR}/abcc-driver-api.cmake)

#-----------------------------------------------------------------------------
# Adding the Anybus CompactCom Driver API related include (.h) files
# to the user host application executable target.
#-----------------------------------------------------------------------------
target_include_directories(raspberry_pi_example_project PRIVATE
  ${ABCC_API_INCLUDE_DIRS}
)

#-----------------------------------------------------------------------------
# Linking the Anybus CompactCom Driver library to the executable target.
#-----------------------------------------------------------------------------
target_link_libraries(raspberry_pi_example_project PRIVATE
   -Wl,--whole-archive abcc_api -Wl,--no-whole-archive PkgConfig::LIBGPIOD
)
