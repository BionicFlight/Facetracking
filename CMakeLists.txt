cmake_minimum_required(VERSION 2.8.3)
project(fv_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    rostime
    roscpp
    rosconsole
    roscpp_serialization
    sensor_msgs
    image_transport
    cv_bridge
)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)
find_package(OpenCV 3.3.1 REQUIRED)

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
 add_message_files(
   FILES
   Diff.msg
 )

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
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs 
# Or other packages containing msgs

 )

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
  INCLUDE_DIRS include
#  LIBRARIES fly_vision
   CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  third-party/id_management
  third-party/kcftracker
  third-party/imagequeue
  ${catkin_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fly_vision.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fly_vision_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


## Libs
add_library(id_management
    third-party/id_management/id_management.h
    third-party/id_management/id_management.cpp
)
target_link_libraries(id_management ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(kcftracker
    third-party/kcftracker/kcftracker.hpp
    third-party/kcftracker/ffttools.hpp
    third-party/kcftracker/fhog.hpp
    third-party/kcftracker/labdata.hpp
    third-party/kcftracker/recttools.hpp
    third-party/kcftracker/tracker.h

    third-party/kcftracker/kcftracker.cpp
    third-party/kcftracker/fhog.cpp
)
target_link_libraries(kcftracker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(imagequeue
    third-party/imagequeue/ImageQueue.h
    third-party/imagequeue/ImageQueueLayout.h
    third-party/imagequeue/QueueHead.h
    third-party/imagequeue/ResourceGuard.h
    third-party/imagequeue/SemaphoreLocker.h
    third-party/imagequeue/traceutil.h
    third-party/imagequeue/util.h

    third-party/imagequeue/ImageQueue.cpp
    third-party/imagequeue/ImageQueueLayout.cpp
    third-party/imagequeue/QueueHead.cpp
    third-party/imagequeue/SemaphoreLocker.cpp
    third-party/imagequeue/util.cpp
)
target_link_libraries(imagequeue ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

## Nodes
add_executable(web_cam
    nodes/web_cam.cpp
)
target_link_libraries(web_cam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management)

add_executable(web_cam_local
    nodes/web_cam_local.cpp
)
target_link_libraries(web_cam_local ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management)

add_executable(tracker_kcf
    nodes/tracker_kcf.cpp
)
target_link_libraries(tracker_kcf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} kcftracker)

add_executable(tracker_kcf_face
    nodes/tracker_kcf_face.cpp
)
target_link_libraries(tracker_kcf_face ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} kcftracker)

add_executable(add_id
    nodes/add_id.cpp
)
target_link_libraries(add_id ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management)


add_executable(read_id
    nodes/read_id.cpp
)
target_link_libraries(read_id ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management)

add_executable(add_memory
    nodes/add_memory.cpp
)
target_link_libraries(add_memory ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management imagequeue
    -lrt -DBUILD_TEST=OFF)

add_executable(read_memory
    nodes/read_memory.cpp
)
target_link_libraries(read_memory ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} imagequeue
    -lrt -DBUILD_TEST=OFF)

add_executable(test_memory
    nodes/test_memory.cpp
)
target_link_libraries(test_memory ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management imagequeue
    -lrt -DBUILD_TEST=OFF)

add_executable(sleep
    nodes/sleep.cpp
)
target_link_libraries(sleep ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(timers
    nodes/timers.cpp
)
target_link_libraries(timers ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(GroundStation_demo
    nodes/GroundStation_demo.cpp
)
target_link_libraries(GroundStation_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} id_management)

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
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fly_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
