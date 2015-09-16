set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FILE(GLOB HEADER_LIST ${PROJECT_SOURCE_DIR}/include/v2uros/*.h)
FILE(GLOB SRC_LIST ${PROJECT_SOURCE_DIR}/src/*.cpp)

find_package(Qt4 COMPONENTS QTCORE QTGUI)
include_directories(${QT_INCLUDES})

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/epiphan_sdk/epiphan/frmgrab/include
                    ${PROJECT_SOURCE_DIR}/epiphan_sdk/epiphan/include
                    ${PROJECT_SOURCE_DIR}/epiphan_sdk/epiphan/samples/v2u_lib/include)

link_directories(${PROJECT_SOURCE_DIR}/epiphan_sdk/epiphan/frmgrab/lib/linux/x86_64/
                 ${PROJECT_SOURCE_DIR}/epiphan_sdk/opensource/slib)

rosbuild_add_executable(${PROJECT_NAME} ${SRC_LIST} ${HEADER_LIST})
target_link_libraries(${PROJECT_NAME} frmgrab slava Qt4::QtCore ${OpenCV_LIBS})
