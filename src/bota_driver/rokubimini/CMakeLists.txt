cmake_minimum_required(VERSION 3.1.3)
project(rokubimini)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++14 -Wall -Werror)

if(CMAKE_COMPILER_IS_GNUCXX) 
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0) 
    add_definitions(-faligned-new)
  endif() 
endif() 

set(PACKAGE_DEPENDENCIES
    roscpp
    sensor_msgs
    geometry_msgs
)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES} 
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    EIGEN3
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/calibration/SensorCalibration.cpp
  src/${PROJECT_NAME}/configuration/Configuration.cpp
  src/${PROJECT_NAME}/configuration/ForceTorqueFilter.cpp
  src/${PROJECT_NAME}/configuration/SensorConfiguration.cpp
  src/${PROJECT_NAME}/fsm/StateEnum.cpp
  src/${PROJECT_NAME}/Reading.cpp
  src/${PROJECT_NAME}/Rokubimini.cpp
  src/${PROJECT_NAME}/Statusword.cpp
  src/${PROJECT_NAME}/calibration/ForceTorqueCalibration.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )
    catkin_add_gtest(
        test_${PROJECT_NAME}
        test/main.cpp
        test/TestConfiguration.cpp
        test/TestReading.cpp
        test/TestStateEnum.cpp
        test/TestStatusWord.cpp
        test/TestRokubimini.cpp
        )
        endif()
        
        if(TARGET test_${PROJECT_NAME})
        target_link_libraries(
            test_${PROJECT_NAME}
            ${PROJECT_NAME}
            )
            endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
add_clang_tooling(
    TARGET ${PROJECT_NAME} test_${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
    )
endif(cmake_clang_tools_FOUND)
