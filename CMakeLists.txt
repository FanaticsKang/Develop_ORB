#
# Author: Kang Yifei
# Email: ccyclonel@126.com
#

cmake_minimum_required( VERSION 2.8 )
project( vSLAM )

#set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O0 -g" )
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
include_directories(include)

find_package(OpenCV REQUIRED)

add_executable(TEST_ORB_EXTRACTOR test/test_orb_extractor.cc src/orb_extractor.cc)
target_link_libraries(TEST_ORB_EXTRACTOR ${OpenCV_LIBRARIES})

add_executable(test_ORB_OPENCV_EXTRACTOR test/compare_orb_opencv_extractor.cc src/orb_extractor.cc)
target_link_libraries(test_ORB_OPENCV_EXTRACTOR ${OpenCV_LIBRARIES})

add_executable(test_STD_COPY test/test_std_copy.cc)
target_link_libraries(test_STD_COPY ${OpenCV_LIBRARIES})

add_executable(test_PYRAMID_RELATED test/test_pyramid_related.cc)
target_link_libraries(test_PYRAMID_RELATED ${OpenCV_LIBRARIES})

add_executable(test_GaussianBlur test/test_GaussianBlur.cc)
target_link_libraries(test_GaussianBlur ${OpenCV_LIBRARIES})

