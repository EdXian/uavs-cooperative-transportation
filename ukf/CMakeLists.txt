cmake_minimum_required(VERSION 2.8.3)
project(ukf)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs diagnostic_msgs diagnostic_updater eigen_conversions geographic_msgs geometry_msgs message_filters message_generation nav_msgs roscpp roslint sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros xmlrpcpp

  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES UKF
#  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater eigen_conversions geographic_msgs geometry_msgs message_filters message_generation nav_msgs roscpp roslint sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros xmlrpcpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(HEADERS_      src/forceest.h src/ukf.h)
set(SOURCES_      src/forceest.cpp src/ukf.cpp)


add_executable(force_estimatee src/force_estimate.cpp ${HEADERS_} ${SOURCES_})

target_link_libraries(force_estimatee
   ${catkin_LIBRARIES}
)


#IF(UKF_FOUND)

#    add_executable(UKF_estimate src/UKF_estimate.cpp ${HEADERS_} ${SOURCES_})
#    #add_executable(payload src/payload.cpp)
#    #add_executable(localization src/localization.cpp)
#    add_executable(leaderr src/leader.cpp ${HEADERS_} ${SOURCES_})
#    add_executable(followerr src/follower.cpp ${HEADERS_} ${SOURCES_})
#    #add_executable(printkeyboard src/printkeyboard.cpp)
#    add_executable(leaderfollower src/leader_follower.cpp ${HEADERS_} ${SOURCES_})
#    add_executable(plotforce src/plotforce.cpp ${HEADERS_} ${SOURCES_})
#    add_executable(formation_three_interaction src/formation_three_interaction.cpp)
#    add_executable(formation_two_interaction src/formation_two_interaction.cpp)
#    #add_executable(estimate_payload src/estimate_payload.cpp)
#    add_executable(estimate_payload_v2 src/estimate_payload_v2.cpp ${HEADERS_} ${SOURCES_})
#    add_executable(light src/light.cpp)
#    add_executable(leader_follower_force src/leader_follower_force.cpp ${HEADERS_} ${SOURCES_})
#    add_executable(estimate_payload_v3 src/estimate_payload_v3.cpp)
#    add_executable(control_rc src/control_rc.cpp)
#    add_executable(leader_acc src/leader_acc.cpp)
#    add_executable(add_random src/add_random.cpp)
#    add_executable(leader_follower_circle src/leader_follower_circle.cpp ${HEADERS_} ${SOURCES_})

#ENDIF()





## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})






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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_UKF.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
