CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(hw_video_analysis)
SET(CMAKE_CXX_FLAGS  ${CMAKE_CXX_FLAGS} "-std=c++11")
SET(ROOT_PATH ./)
SET(PATH_3RDPARTY 3rdparty)
INCLUDE(cmake/3rdparty.cmake)
INCLUDE(cmake/config.cmake)
INCLUDE_DIRECTORIES(include src)
FILE(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c" "src/*.cc"
        "include/*.hpp" "include/*.h"
        "src/*.hpp" "src/*.h")
#设定包含的模块
#P_SAMPLE_INCLUDE_MODULES(${sample} MODULES dummy)
# 依赖库之间能够自己解决依赖问题（找到直接依赖的模块即可）
SET(DEPS ${GLOG_LIBS} ${COMMON_LIBS} ${CZML_LIBS}
        ${TOOLBOX_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS})
ADD_EXECUTABLE(hw_video_analysis hw_video_analysis.cpp ${SRCS})
TARGET_LINK_LIBRARIES(hw_video_analysis ${DEPS})