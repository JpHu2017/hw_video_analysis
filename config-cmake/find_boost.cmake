SET(LIBRARY_NAME boost)
SET(LIBRARY_NAME_UPPER BOOST)
SET(LIBRARY_VERSION boost-1.58.0)

MESSAGE("Get ${LIBRARY_VERSION} From System")
FIND_PACKAGE(Boost 1.58 REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS}/include)
SET(${LIBRARY_NAME_UPPER}_LIBS boost_atomic boost_chrono
            boost_date_time boost_filesystem boost_python
            boost_program_options  boost_system boost_thread )

