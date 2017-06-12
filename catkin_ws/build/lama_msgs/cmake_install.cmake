# Install script for directory: /home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lama_msgs/msg" TYPE FILE FILES
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg/Crossing.msg"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg/DescriptorLink.msg"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg/Frontier.msg"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg/InterfaceInfo.msg"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg/LamaObject.msg"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg/PlaceProfile.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lama_msgs/srv" TYPE FILE FILES
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/GetCrossing.srv"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/SetCrossing.srv"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/GetLamaObject.srv"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/SetLamaObject.srv"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/GetPlaceProfile.srv"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/SetPlaceProfile.srv"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/srv/GetInterfaceInfo.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lama_msgs/cmake" TYPE FILE FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/build/lama_msgs/catkin_generated/installspace/lama_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/devel/include/lama_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/devel/share/roseus/ros/lama_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/devel/share/common-lisp/ros/lama_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/devel/share/gennodejs/ros/lama_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/devel/lib/python2.7/dist-packages/lama_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/devel/lib/python2.7/dist-packages/lama_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/build/lama_msgs/catkin_generated/installspace/lama_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lama_msgs/cmake" TYPE FILE FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/build/lama_msgs/catkin_generated/installspace/lama_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lama_msgs/cmake" TYPE FILE FILES
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/build/lama_msgs/catkin_generated/installspace/lama_msgsConfig.cmake"
    "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/build/lama_msgs/catkin_generated/installspace/lama_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lama_msgs" TYPE FILE FILES "/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/package.xml")
endif()

