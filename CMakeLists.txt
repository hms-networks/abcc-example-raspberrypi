# This CMake script is written to work with minimum CMake version 3.10.
cmake_minimum_required(VERSION 3.12)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

# Creating a user host application project.
project(raspberry_pi_example_project C)

# Sanity check that this is ran in a supported environment.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
else()
   message(WARNING "Host ${CMAKE_SYSTEM_NAME} is not supported to build raspberry_pi_example_project.")
endif()


# Find the required library
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# Creating a user host application executable target.
add_executable(raspberry_pi_example_project
   ${PROJECT_SOURCE_DIR}/src/main.c
)

# Source (.c) files to add to the executable target. These are 'user unique'. Some are
# related to the CompactCom Driver API.
set(raspberry_pi_example_SRCS
   ${PROJECT_SOURCE_DIR}/src/main.c
   ${PROJECT_SOURCE_DIR}/src/example_application/abcc_network_data_parameters.c
   ${PROJECT_SOURCE_DIR}/src/example_application/implemented_callback_functions.c
   ${PROJECT_SOURCE_DIR}/src/example_application/cyclical_processing.c
   ${PROJECT_SOURCE_DIR}/src/example_application/Logprint/logprint.c
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_hardware_abstraction.c
)

# Include (.h) files related to the user host application.
set(raspberry_pi_example_INCS
   ${PROJECT_SOURCE_DIR}/src/example_application/abcc_network_data_parameters.h
   ${PROJECT_SOURCE_DIR}/src/example_application/Logprint/logprint.h
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_driver_config.h
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_software_port.h
   ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/abcc_types.h
)

# Adding the 'user unique' sources to the user host application executable target.
# (The include files are added to keep the file and directory tree structure.)
target_sources(raspberry_pi_example_project PRIVATE
   ${raspberry_pi_example_SRCS}
   ${raspberry_pi_example_INCS}
)

# Directories containing 'user unique' include (.h) files related to the CompactCom
# Driver API.
set(ABCC_API_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/src/abcc_adaptation/
  ${PROJECT_SOURCE_DIR}/src/example_application/
  ${PROJECT_SOURCE_DIR}/src/example_application/Logprint/
)

# The directory containing the Anybus CompactCom Driver API repository.
set(ABCC_API_DIR ${PROJECT_SOURCE_DIR}/lib/abcc-driver-api)

# Including the Anybus CompactCom Driver API's CMake module file.
include(${ABCC_API_DIR}/abcc-driver-api.cmake)

# Adding both the Anybus CompactCom Driver API related and unrelated include (.h) files
# to the user host application executable target.
target_include_directories(raspberry_pi_example_project PRIVATE
  ${LIBGPIOD_LIBRARIES}
  ${ABCC_API_INCLUDE_DIRS}
)

#add_dependencies(raspberry_pi_example_project abcc_api)

# Linking the Anybus CompactCom Driver library to the executable target.
target_link_libraries(raspberry_pi_example_project PRIVATE abcc_api ${LIBGPIOD_LIBRARIES})
