cmake_minimum_required(VERSION 3.0.2)
project(bota_driver_testing)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(BUILD_HW_TESTS "Build the hardware tests" OFF)
set(PACKAGE_DEPENDENCIES
  bota_node
  rokubimini
  rokubimini_serial
  rokubimini_ethercat
  rokubimini_bus_manager
  rostest
  rosgraph_msgs
  geometry_msgs
  rokubimini_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

add_definitions(-std=c++14 -Wall -Werror)

if(CMAKE_COMPILER_IS_GNUCXX) 
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0) 
    add_definitions(-faligned-new)
  endif() 
endif() 

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${PACKAGE_DEPENDENCIES}
)

include_directories(${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  if(BUILD_HW_TESTS)
    include_directories(${catkin_INCLUDE_DIRS})
    add_rostest_gtest(
        ${PROJECT_NAME}_boot_sync
        launch/${PROJECT_NAME}_boot_sync.test
        src/main.cpp
        src/TestBotaDriverBoot.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_boot_async
        launch/${PROJECT_NAME}_boot_async.test
        src/main.cpp
        src/TestBotaDriverBoot.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_reset_service_sync
        launch/${PROJECT_NAME}_reset_service_sync.test
        src/main.cpp
        src/TestBotaDriverResetService.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_reset_service_async
        launch/${PROJECT_NAME}_reset_service_async.test
        src/main.cpp
        src/TestBotaDriverResetService.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_ft_offsets_sync
        launch/${PROJECT_NAME}_ft_offsets_sync.test
        src/main.cpp
        src/TestBotaDriverFTOffsets.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_ft_offsets_async
        launch/${PROJECT_NAME}_ft_offsets_async.test
        src/main.cpp
        src/TestBotaDriverFTOffsets.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_zero_measurements_sync
        launch/${PROJECT_NAME}_zero_measurements_sync.test
        src/main.cpp
        src/TestBotaDriverZeroMeasurements.cpp
    )
    add_rostest_gtest(
        ${PROJECT_NAME}_zero_measurements_async
        launch/${PROJECT_NAME}_zero_measurements_async.test
        src/main.cpp
        src/TestBotaDriverZeroMeasurements.cpp
    )
    add_rostest(launch/${PROJECT_NAME}_param_sync.test)
    add_rostest(launch/${PROJECT_NAME}_param_async.test)
    add_rostest(launch/${PROJECT_NAME}_hz_512_sync.test)
    add_rostest(launch/${PROJECT_NAME}_hz_512_async.test)
    add_rostest(launch/${PROJECT_NAME}_hz_256_sync.test)
    add_rostest(launch/${PROJECT_NAME}_hz_256_async.test)
    add_rostest(launch/${PROJECT_NAME}_hz_128_sync.test)
    add_rostest(launch/${PROJECT_NAME}_hz_128_async.test)
    add_rostest(launch/${PROJECT_NAME}_hz_64_sync.test)
    add_rostest(launch/${PROJECT_NAME}_hz_64_async.test)
  endif()
endif()
if(TARGET ${PROJECT_NAME}_boot_sync)
  target_link_libraries(
    ${PROJECT_NAME}_boot_sync
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_boot_async)
  target_link_libraries(
    ${PROJECT_NAME}_boot_async
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_reset_service_sync)
  target_link_libraries(
    ${PROJECT_NAME}_reset_service_sync
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_reset_service_async)
  target_link_libraries(
    ${PROJECT_NAME}_reset_service_async
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_ft_offsets_sync)
  target_link_libraries(
    ${PROJECT_NAME}_ft_offsets_sync
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_ft_offsets_async)
  target_link_libraries(
    ${PROJECT_NAME}_ft_offsets_async
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_zero_measurements_sync)
  target_link_libraries(
    ${PROJECT_NAME}_zero_measurements_sync
    ${catkin_LIBRARIES}
  )
endif()
if(TARGET ${PROJECT_NAME}_zero_measurements_async)
  target_link_libraries(
    ${PROJECT_NAME}_zero_measurements_async
    ${catkin_LIBRARIES}
  )
endif()
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGET
    ${PROJECT_NAME}
    test_${PROJECT_NAME}
    SOURCE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    CF_FIX)
endif(cmake_clang_tools_FOUND)
