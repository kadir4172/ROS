cmake_minimum_required(VERSION 2.8.3)
project(a3_help)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_definitions( -DANALYSIS=ON )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
add_service_files(
   FILES
   RequestGoal.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs#   std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a3_help
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(a3_help
#   src/${PROJECT_NAME}/a3_help.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(a3_help ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(gazebo_retrieve src/gazebo_retrieve.cpp)
add_executable(map_to_image_node src/map_to_image_node.cpp)
add_executable(sample src/sample.cpp)
add_executable(gen_path src/gen_path.cpp)
add_executable(TBK src/TBK.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(a3_help_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(gazebo_retrieve ${catkin_LIBRARIES})
target_link_libraries(map_to_image_node ${catkin_LIBRARIES})
target_link_libraries(sample ${catkin_LIBRARIES})
target_link_libraries(gen_path ${catkin_LIBRARIES})
target_link_libraries(TBK ${catkin_LIBRARIES})

#target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp src/raytracer.cpp)
catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp )
if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
