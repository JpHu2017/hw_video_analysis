####
# Set Python INCLUDE DIRS & LIBS
SET(PYTHON_INCLUDE_DIRS /usr/include/python2.7)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
SET(PYTHON_LIBS python2.7)
####
# set boost_python lib
SET(BOOST_PYTHON_LIBS boost_python)
SET(BOOST_SYSTEM_LIBS boost_system)
# set pthread
SET(PTHREAD_LIBS pthread)
# set cblas
SET(CBLAS_LIBS cblas)
