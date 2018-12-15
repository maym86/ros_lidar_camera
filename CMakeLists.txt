cmake_minimum_required(VERSION 2.8)
project( vo )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( OpenCV 3 REQUIRED )

find_package( Boost 1.58 REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

find_package(gflags REQUIRED)
find_package(glog REQUIRED)

include_directories(include)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable( vo ${SRC_FILES})

target_link_libraries( vo ${OpenCV_LIBS} ${Boost_LIBRARIES} gflags glog)

# Setup testing
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

file(GLOB TEST_FILES ${PROJECT_SOURCE_DIR}/src/test/*.cpp)

# Add test cpp file
#add_executable(unit_tests ${TEST_FILES})

#target_link_libraries(unit_tests ${OpenCV_LIBS} ${Boost_LIBRARIES} gflags glog gtest gtest_main)
#add_test( unit_tests unit_tests )
