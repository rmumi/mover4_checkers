cmake_minimum_required(VERSION 2.8.3)
project(checkers_main)


find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs geometry_msgs)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(src/hello_world src/build-hello_world-Desktop_Qt_5_8_0_GCC_64bit-Debug)

qt5_wrap_cpp(QT_MOC src/hello_world/mainwindow.h src/build-hello_world-Desktop_Qt_5_8_0_GCC_64bit-Debug/ui_mainwindow.h)
set(SOURCES src/hello_world/main.cpp src/hello_world/mainwindow.cpp ${QT_MOC})

add_library(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
