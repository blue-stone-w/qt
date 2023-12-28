cmake_minimum_required(VERSION 3.10)
project(map_visual)
# set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)
# include(FindLibXml2 REQUIRED)
find_package( REQUIRED COMPONENTS
        roscpp
        rosconsole
        std_msgs
        sensor_msgs
        glviewer
        rosinterface
        cv_bridge
        pcl_ros
        image_transport
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall -g ${CMAKE_CXX_FLAGS}")

EXECUTE_PROCESS(
        COMMAND pkg-config --variable=host_bins Qt5Core
        OUTPUT_VARIABLE Qt5BIN
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${LIBXML2_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${GLUT_INCLUDE_DIRS}
)

qt5_wrap_cpp(MOC_FILES include/mainwindow.h)

# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(map_visual 
              src/map_visual.cpp
              src/mainwindow.cpp
              ${MOC_FILES}
              )

# add_dependencies(calibration_publisher        ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_visual 
                ${OPENGL_LIBRARIES}
                ${PCL_LIBRARIES}
                ${GLUT_LIBRARY}
                Qt5::Core
                Qt5::Gui
                Qt5::Widgets
                Qt5::OpenGL)
