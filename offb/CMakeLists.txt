cmake_minimum_required(VERSION 2.8.3)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  mav_msgs
  #roscpp
  #gazebo_msgs
  #UKF
)
find_package(gazebo REQUIRED)
find_package(ompl)
find_package(
    PythonLibs
    )




#IF (gazebo_msgs_FOUND)


#ENDIF


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#DEPENDS
#   roscpp
#   gazebo_ros

)

###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    usr/include
    usr/include/gazebo-9
    ${OMPL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library




add_library(
    geometric_controller  SHARED
    include/geometric_controller.cpp
)
add_library(lpfilter   SHARED include/lpf.cpp)
add_library(qptrajectory SHARED include/qptrajectory.cpp)

target_link_libraries(
    qptrajectory
      ${catkin_LIBRARIES}
    )
#add_library(world_plugin src/world_plugin.cpp)





#target_link_libraries(world_plugin
#    ${catkin_LIBRARIES}
#    ${GAZEBO_LIBRARIES}

#    )

add_library(
    rls  SHARED
     include/rls.cpp
 )




## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

#add_executable(attb src/attb.cpp)
#add_executable(drone2 src/drone2.cpp)
#add_executable(thrust src/thrust.cpp)
#add_executable(apply_wrench src/apply_wrench.cpp)
add_executable(geo src/geo.cpp)
add_executable(mocap src/mocap.cpp)
add_executable(geo_plan src/geo_plan.cpp)
add_executable(geo2 src/geo2.cpp)
add_executable(obstacle src/obstacle.cpp)
add_executable(geo2_new src/geo2_new.cpp  )
#add_executable(force src/force.cpp)
#add_executable(coop1 src/coop1.cpp)

#add_executable(attc src/attc.cpp)
#add_executable(multidrone src/multidrone.cpp)
#add_executable(drone1 src/drone1.cpp)
#add_executable(body src/body.cpp)
#add_executable(setmode src/setmode.cpp)



#add_executable(tag src/tag.cpp)
#  add_executable(rotors src/rotors.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(
#    force
#     ${catkin_LIBRARIES}

#)
#add_dependencies(

#    thrust
#    rls
#    )
#target_link_libraries(thrust
#   ${catkin_LIBRARIES}
#   ${PYTHON_LIBRARIES}
#   qptrajectory
#    rls
#   )
target_link_libraries(
    rls
     ${catkin_LIBRARIES}

)
target_link_libraries(
    mocap
     ${catkin_LIBRARIES}

)
#target_link_libraries(
#    coop1
#     ${catkin_LIBRARIES}
#     ${PYTHON_LIBRARIES}
#     qptrajectory
#     geometric_controller

#    )
target_link_libraries(
   obstacle
     ${catkin_LIBRARIES}
     ${PYTHON_LIBRARIES}
     qptrajectory
     geometric_controller
     ${OMPL_LIBRARIES}
)
target_link_libraries(
    geo
     ${catkin_LIBRARIES}
     ${PYTHON_LIBRARIES}
     qptrajectory
     geometric_controller
)
target_link_libraries(
    geo_plan
     ${catkin_LIBRARIES}
     ${PYTHON_LIBRARIES}
     qptrajectory
     geometric_controller
)
target_link_libraries(
    geo2
     ${catkin_LIBRARIES}
     ${PYTHON_LIBRARIES}
     qptrajectory
     geometric_controller
)
target_link_libraries(
    geo2_new
     ${catkin_LIBRARIES}
     ${PYTHON_LIBRARIES}
     qptrajectory
     geometric_controller
     lpfilter
)



#target_link_libraries(attb
#   ${catkin_LIBRARIES}
#   ${PYTHON_LIBRARIES}
#   qptrajectory
#     geometric_controller
#   )
#target_link_libraries(drone2
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(apply_wrench
#  ${catkin_LIBRARIES}
#)

#catkin_package(
#  DEPENDS
#    roscpp
#    gazebo_ros
#)
#target_link_libraries(attc
#   ${catkin_LIBRARIES}
#   ${PYTHON_LIBRARIES}
#   qptrajectory

#   )


#target_link_libraries(

#    setmode
#    ${catkin_LIBRARIES}

#    )

#target_link_libraries(multidrone
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(drone1
#  ${catkin_LIBRARIES}
#  ${PYTHON_LIBRARIES}
#   qptrajectory
#)


#target_link_libraries(body
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(rotors
#  ${catkin_LIBRARIES}
#)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_offb.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
