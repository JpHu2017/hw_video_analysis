# Config file for the Common package.
#
# After successful configuration the following variables
# will be defined:
#
#   Common_INCLUDE_DIRS - Caffe include directories
#   Common_LIBS    - libraries to link against


# Compute paths
get_filename_component(Common_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(Common_INCLUDE_DIRS "${Common_CMAKE_DIR}/../../include")
FIND_LIBRARY(Common_LIBS NAMES libcz_utils.so PATHS "${Common_CMAKE_DIR}/../../lib")
