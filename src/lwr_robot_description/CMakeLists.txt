#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(lwr_robot_description)
find_package(catkin REQUIRED COMPONENTS
  rospy
  urdf
  xacro
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)


catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  rospy
  geometry_msgs

)

file(GLOB DEPLOY_FILES_AND_DIRS "${PROJECT_SOURCE_DIR}/*")
foreach(ITEM ${DEPLOY_FILES_AND_DIRS})
   get_filename_component(tmpName ${ITEM} NAME)
   SET(tmpName2 "${PROJECT_SOURCE_DIR}/${tmpName}")
   
   IF( IS_DIRECTORY "${ITEM}" )
      IF( NOT ((${ITEM} MATCHES "build") OR (${ITEM} MATCHES ".git")))
        LIST( APPEND DIRS_TO_DEPLOY "${ITEM}" )
      ENDIF()
   ELSE()
      IF( NOT ((${ITEM} MATCHES "CMakeLists.txt") OR (${ITEM} MATCHES "README.md")))
        LIST( APPEND FILES_TO_DEPLOY "${ITEM}" )
      ENDIF()
   ENDIF()
endforeach()
INSTALL( FILES ${FILES_TO_DEPLOY} DESTINATION etc/lwr_robot_description)
INSTALL( DIRECTORY ${DIRS_TO_DEPLOY} DESTINATION etc/lwr_robot_description)
