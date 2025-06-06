cmake_minimum_required(VERSION 3.5.1)
project(astra_camera)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g")
set(CMAKE_BUILD_TYPE "Release")
option(USE_RK_MPP "Use rock chip MPP libary for hardware decode" OFF)
option(USE_RK_RGA "Use rock chip RGA libary for hardware decode" OFF)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(catkin REQUIRED
  std_msgs
  geometry_msgs
  backward_ros
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  message_filters
  camera_info_manager
  sensor_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_search_module(LIBUVC REQUIRED libuvc)
if (NOT LIBUVC_FOUND)
  message(FATAL_ERROR "libuvc is not found")
endif ()

if (USE_RK_MPP)
  message(STATUS "Use rock chip MPP libary for hardware decode")
  add_definitions(-DUSE_RK_MPP)
  pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
  if (NOT RK_MPP_FOUND)
    message(FATAL_ERROR "rockchip_mpp is not found")
  endif ()
endif ()
if (USE_RK_RGA)
  message(STATUS "Use rock chip RGA libary for hardware decode")
  add_definitions(-DUSE_RK_RGA)
  pkg_search_module(RK_RGA REQUIRED librga)
  if (NOT RK_RGA_FOUND)
    message(FATAL_ERROR "rockchip_rga is not found")
  endif ()
endif ()
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm")
  set(HOST_PLATFORM "arm")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm")
endif ()

message(STATUS "ORRBEC : ${HOST_PLATFORM}")
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

generate_dynamic_reconfigure_options(cfg/Astra.cfg)

add_message_files(
  FILES
  DeviceInfo.msg
  Extrinsics.msg
  Metadata.msg
  LabelXYZ.msg
)

add_service_files(
  FILES
  GetBool.srv
  GetCameraInfo.srv
  GetCameraParams.srv
  GetDeviceInfo.srv
  GetDouble.srv
  GetInt32.srv
  GetString.srv
  SetInt32.srv
  SetString.srv
  DetectObject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs
) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  message_runtime
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  std_srvs
  message_filters
  camera_info_manager
  backward_ros
  geometry_msgs
)

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})

set(ORBBEC_OPENNI2_REDIST
  "${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/${HOST_PLATFORM}")

set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH}:${ORBBEC_OPENNI2_REDIST}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ORBBEC_OPENNI2_REDIST}")

add_library(${PROJECT_NAME}
  src/point_cloud_proc/point_cloud_xyz.cpp
  src/point_cloud_proc/point_cloud_xyzrgb.cpp
  src/d2c_viewer.cpp
  src/ob_context.cpp
  src/ob_camera_info.cpp
  src/ob_camera_node_factory.cpp
  src/ob_camera_node.cpp
  src/ros_service.cpp
  src/utils.cpp
  src/uvc_camera_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${libuvc_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
  ${RK_MPP_INCLUDE_DIRS}
  ${RK_RGA_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  -lOpenNI2
  -ldw # for stacktrace
  -L${ORBBEC_OPENNI2_REDIST}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${LIBUVC_LIBRARIES}
  ${RK_MPP_LIBRARIES}
  ${RK_RGA_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBUVC_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  -lOpenNI2
  -L${ORBBEC_OPENNI2_REDIST}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${LIBUVC_LIBRARIES}
)

add_executable(list_devices_node
  src/list_devices_node.cpp
)
target_include_directories(list_devices_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBUVC_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
)

target_link_libraries(list_devices_node
  -lOpenNI2
  -L${ORBBEC_OPENNI2_REDIST}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${LIBUVC_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  list_devices_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${ORBBEC_OPENNI2_REDIST}/libOpenNI2.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)

install(DIRECTORY ${ORBBEC_OPENNI2_REDIST}/OpenNI2
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*"
  PATTERN "openni2_redist" EXCLUDE
)

install(FILES 56-orbbec-usb.rules
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)

install(FILES 56-orbbec-usb.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
