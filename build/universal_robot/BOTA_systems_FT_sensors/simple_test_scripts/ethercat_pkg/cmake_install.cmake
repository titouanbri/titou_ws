# Install script for directory: /home/titouan/catkin_ws/src/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/titouan/catkin_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/ethercat_pkg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ethercat_pkg/cmake" TYPE FILE FILES
    "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/ethercat_pkgConfig.cmake"
    "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/ethercat_pkgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ethercat_pkg" TYPE FILE FILES "/home/titouan/catkin_ws/src/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ethercat_pkg" TYPE PROGRAM FILES "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/publisher_ethercat.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ethercat_pkg" TYPE PROGRAM FILES "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/bota_ethercat_basic_example.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ethercat_pkg" TYPE PROGRAM FILES "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/bota_ethercat_minimal_example.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ethercat_pkg" TYPE PROGRAM FILES "/home/titouan/catkin_ws/build/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/catkin_generated/installspace/launcher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ethercat_pkg" TYPE PROGRAM FILES "/home/titouan/catkin_ws/src/universal_robot/BOTA_systems_FT_sensors/simple_test_scripts/ethercat_pkg/scripts/start_ethercat.sh")
endif()

