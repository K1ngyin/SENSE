cmake_minimum_required(VERSION 3.8)
project(astra_camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio)
find_package(Threads REQUIRED)

# The installer normally places OpenNI2 in the system paths. Also search the
# persistent SDK cache created by camera/install_astra_openni.sh.
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(OPENNI2_BUNDLE_ROOT "${PROJECT_ROOT}/camera/.sdk_cache/arm64_sdk")

find_path(OPENNI2_INCLUDE_DIR
  NAMES OpenNI.h
  HINTS
    /usr/include/openni2
    /usr/local/include/openni2
    /usr/include
    /usr/local/include
    "${OPENNI2_BUNDLE_ROOT}"
  PATH_SUFFIXES openni2 Include Include/openni2
)

find_library(OPENNI2_LIBRARY
  NAMES OpenNI2
  HINTS
    /usr/lib
    /usr/local/lib
    /usr/lib/aarch64-linux-gnu
    "${OPENNI2_BUNDLE_ROOT}"
  PATH_SUFFIXES lib lib64 Redist
)

if(NOT OPENNI2_INCLUDE_DIR)
  file(GLOB_RECURSE _openni_headers "${OPENNI2_BUNDLE_ROOT}/*/OpenNI.h")
  if(_openni_headers)
    list(GET _openni_headers 0 _openni_header)
    get_filename_component(OPENNI2_INCLUDE_DIR "${_openni_header}" DIRECTORY)
  endif()
endif()

if(NOT OPENNI2_LIBRARY)
  file(GLOB_RECURSE _openni_libraries
    "${OPENNI2_BUNDLE_ROOT}/*/libOpenNI2.so"
    "${OPENNI2_BUNDLE_ROOT}/*/libOpenNI2.so.*"
  )
  if(_openni_libraries)
    list(GET _openni_libraries 0 OPENNI2_LIBRARY)
  endif()
endif()

if(NOT OPENNI2_INCLUDE_DIR OR NOT OPENNI2_LIBRARY)
  message(FATAL_ERROR
    "OpenNI2 headers/library not found. Run bash camera/install_astra_openni.sh first.")
endif()

# OpenNI loads device plugins from <redist>/OpenNI2/Drivers. Prefer the
# installed path; fall back to the bundled ARM64 SDK tree.
set(OPENNI2_REDIST_DIR "")
set(_openni_driver_dirs
  "/usr/lib/OpenNI2/Drivers"
  "/usr/lib/aarch64-linux-gnu/OpenNI2/Drivers"
  "/usr/local/lib/OpenNI2/Drivers"
)
file(GLOB_RECURSE _bundled_openni_driver_dirs LIST_DIRECTORIES TRUE
  "${OPENNI2_BUNDLE_ROOT}/*/OpenNI2/Drivers"
  "${OPENNI2_BUNDLE_ROOT}/*/Redist/OpenNI2/Drivers"
)
list(APPEND _openni_driver_dirs ${_bundled_openni_driver_dirs})
foreach(_driver_dir IN LISTS _openni_driver_dirs)
  if(IS_DIRECTORY "${_driver_dir}")
    get_filename_component(_openni2_dir "${_driver_dir}" DIRECTORY)
    get_filename_component(OPENNI2_REDIST_DIR "${_openni2_dir}" DIRECTORY)
    break()
  endif()
endforeach()

if(NOT OPENNI2_REDIST_DIR)
  message(WARNING "OpenNI2/Drivers directory not found during configure; the node may need OPENNI2_REDIST set at runtime.")
endif()

add_executable(astra_camera_node src/astra_camera_node.cpp)
target_include_directories(astra_camera_node PRIVATE
  "${OPENNI2_INCLUDE_DIR}"
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(astra_camera_node
  "${OPENNI2_LIBRARY}"
  ${OpenCV_LIBRARIES}
  Threads::Threads
)
ament_target_dependencies(astra_camera_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  camera_info_manager
)
if(OPENNI2_REDIST_DIR)
  target_compile_definitions(astra_camera_node PRIVATE
    OPENNI2_REDIST_DEFAULT=\"${OPENNI2_REDIST_DIR}\"
  )
endif()
get_filename_component(OPENNI2_LIBRARY_DIR "${OPENNI2_LIBRARY}" DIRECTORY)
set_target_properties(astra_camera_node PROPERTIES
  BUILD_RPATH "${OPENNI2_LIBRARY_DIR}"
  INSTALL_RPATH "${OPENNI2_LIBRARY_DIR}"
)

install(TARGETS astra_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
