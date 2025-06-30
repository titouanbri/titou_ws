cmake_minimum_required(VERSION 3.0.2)
project(rokubimini_serial)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++14 -Wall -Werror)

set(PACKAGE_DEPENDENCIES 
  rokubimini
  rokubimini_bus_manager
  rokubimini_msgs
)

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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/RokubiminiSerial.cpp
  src/${PROJECT_NAME}/RokubiminiSerialImpl.cpp
  src/${PROJECT_NAME}/RokubiminiSerialBusManager.cpp
  src/${PROJECT_NAME}/RokubiminiSerialCommunication.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_bus_manager_node src/${PROJECT_NAME}/${PROJECT_NAME}_bus_manager_node.cpp)
add_dependencies(${PROJECT_NAME}_bus_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_bus_manager_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_bus_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)