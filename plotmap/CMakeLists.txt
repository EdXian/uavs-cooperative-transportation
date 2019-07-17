#https://gist.github.com/gicmo/e394737efa88fcf0dae6 cmake skeleton

cmake_minimum_required(VERSION 3.1.0) #version 3.1.0 is recommended for qt5
project(plotmap)


add_compile_options(-std=c++11)
find_package(OpenGL REQUIRED)
add_definitions(-DQCUSTOMPLOT_USE_OPENGL )
find_package(
    PythonLibs
    )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2
 # ukf_estimate
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport)
find_package(Qt5Core REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(  qt_srcs
      include/mainwindow.cpp
      include/qcustomplot.cpp
   )
set(
    qt_hdrs
    include/mainwindow.h
    include/qcustomplot.h
   )
find_package(Qt5PrintSupport)
#QT5_WRAP_CPP(qt_moc_srcs ${qt_hdrs})


include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Qt5Widgets_INCLUDES})
include_directories(
    /
  include
  ${PYTHON_INCLUDE_DIR}

  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ros_qt_template
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


#tp=turtlepose
add_library(qpsolver include/qptrajectory.cpp)
target_link_libraries (qpsolver  ${PYTHON_LIBRARIES})

add_library(plotmaplib include/mainwindow.cpp)
target_link_libraries (plotmaplib Qt5::Widgets Qt5::Core ${catkin_LIBRARIES} )

add_executable(plotmap src/plotmap.cpp
                                ${qt_srcs}
                                 ${qt_hdrs}
                                 )

add_executable(plot_rviz src/plot_rviz.cpp     )



target_link_libraries(plot_rviz
                  ${catkin_LIBRARIES}
                   )

target_link_libraries(plotmap
                   plot
                   ${OPENGL_LIBRARIES}
                   ${catkin_LIBRARIES}
                   ${Qt5Widgets_LIBRARIES}
                   ${Qt5Core_LIBRARIES}
                    Qt5::PrintSupport
                    ${PYTHON_LIBRARIES}
                    qpsolver
                    )
 #target_link_libraries(subtest ${catkin_LIBRARIES})

