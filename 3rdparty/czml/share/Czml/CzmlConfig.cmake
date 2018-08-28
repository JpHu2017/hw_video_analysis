# Config file for the Czml package.
#
# After successful configuration the following variables
# will be defined:
#
#   Czml_INCLUDE_DIRS - Caffe include directories
#   Czml_LIBS    - libraries to link against


# Compute paths
get_filename_component(Czml_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(Czml_INCLUDE_DIRS "${Czml_CMAKE_DIR}/../../include")
FIND_LIBRARY(Czml_LIBS NAMES libcz_ml_world.so PATHS "${Czml_CMAKE_DIR}/../../lib")
#FILE(GLOB_RECURSE Czml_LIBS "${Czml_CMAKE_DIR}/../../lib/*.so")

