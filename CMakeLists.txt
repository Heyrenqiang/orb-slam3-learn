cmake_minimum_required(VERSION 3.2)
project(test)
find_package(OpenCV 4.0)
MESSAGE(${OpenCV_VERSION})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(ORBSLAM SHARED
src/ORBFeature.cpp
src/Frame.cpp
src/DataIO.cpp
src/Track.cpp
src/MultipleViewGeometry.cpp
src/SlamProcess.cpp
src/Matcher.cpp
include/ORBFeature.hpp
include/Frame.hpp
include/DataIO.hpp
include/Track.hpp
include/MultipleViewGeometry.hpp
include/SlamProcess.hpp
include/Matcher.hpp
)
target_link_libraries(ORBSLAM
${OpenCV_LIBS}
)

add_executable(test_opencv src/test.cpp)
target_link_libraries(test_opencv 
${OpenCV_LIBS}
ORBSLAM
)