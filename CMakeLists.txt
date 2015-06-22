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



find_package(Qt4 COMPONENTS QTCORE QTGUI)
include_directories(${QT_INCLUDES})

#set(QT_USE_QTGUI TRUE)

find_package( OpenCV REQUIRED )

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#find_package(Qt5Widgets)
#include_directories(${Qt5Widgets_INCLUDE_DIRS})

include_directories(./include
                    /home/danying/workspace/epiphan_sdk/epiphan/frmgrab/include
                    /home/danying/workspace/epiphan_sdk/epiphan/include
                    /home/danying/workspace/epiphan_sdk/epiphan/samples/v2u_lib/include
                    )

FILE(GLOB HEADER_LIST include/*.h)
FILE(GLOB HEADER_LIST include/v2uros/*.h)


LINK_DIRECTORIES(/home/danying/workspace/epiphan_sdk/epiphan/frmgrab/lib/linux/x86_64/
                 /home/danying/workspace/epiphan_sdk/opensource/slib
                 )


#aux_source_directory(. SRC_LIST)
aux_source_directory(./src SRC_LIST)

rosbuild_add_executable(${PROJECT_NAME} ${SRC_LIST} ${HEADER_LIST})
target_link_libraries(${PROJECT_NAME} frmgrab slava Qt4::QtCore ${OpenCV_LIBS} ${QT_QTGUI_LIBRARY})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
