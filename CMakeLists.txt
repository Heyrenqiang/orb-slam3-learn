cmake_minimum_required(VERSION 3.2)
project(test)
find_package(OpenCV 4.0)
MESSAGE(${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)

include(cmake/DBoW3.cmake)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIRS}
)
set(ALL_TARGET_LIBRARIES "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(ORBSLAM SHARED
src/ORBFeature.cpp
src/Frame.cpp
src/DataIO.cpp
src/Track.cpp
src/MultipleViewGeometry.cpp
src/SlamProcess.cpp
src/Matcher.cpp
src/Param.cpp
# src/KeyFrame.cpp
src/Map.cpp
src/MapPoint.cpp
src/MapAtlas.cpp
src/Optimizer.cpp
src/LocalMap.cpp
src/Converter.cpp
include/ORBFeature.hpp
include/Frame.hpp
include/DataIO.hpp
include/Track.hpp
include/MultipleViewGeometry.hpp
include/SlamProcess.hpp
include/Matcher.hpp
include/Param.hpp
# include/KeyFrame.hpp
include/Map.hpp
include/MapPoint.hpp
include/MapAtlas.hpp
include/Optimizer.hpp
include/LocalMap.hpp
include/Typedef.hpp
include/Converter.hpp
)
target_link_libraries(ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${DBoW3_LIBS}
)

add_executable(test src/test.cpp)
target_link_libraries(test
${OpenCV_LIBS}
ORBSLAM
)