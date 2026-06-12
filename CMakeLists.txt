#--------------------------------------------------------------
# Project setup
#--------------------------------------------------------------
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

project(masonry)

#--------------------------------------------------------------
# Chrono
# The following Chrono modules can be requested (case insensitive):
#   Cascade, CSharp, DEM, FMI, FSI, FSI_SPH, FSI_TDPF, Irrlicht, OpenGL, VSG, Matlab,
#   Modal, Multicore, PardisoMKL, Parsers, Postprocess, Sensor,
#   Synchrono, Vehicle, VehicleCosim.
#--------------------------------------------------------------
find_package(Chrono
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS
             CONFIG)

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Targets
#--------------------------------------------------------------
add_executable(masonry masonry.cpp)
target_compile_definitions(masonry PRIVATE "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
if(MSVC)
    set_target_properties(masonry PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
target_link_libraries(masonry PRIVATE ${CHRONO_TARGETS})

#--------------------------------------------------------------
# Utils
#--------------------------------------------------------------
add_CHRONO_DLLS_copy_command()

# Create custom commands, invoked pre-build to copy DLLs to the appropriate
# directory (depending on the configuration selected at build time in VS)
add_custom_target(COPY_DATA ALL)

add_custom_command(
    TARGET COPY_DATA PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/data/" "${CMAKE_BINARY_DIR}/data/"
)