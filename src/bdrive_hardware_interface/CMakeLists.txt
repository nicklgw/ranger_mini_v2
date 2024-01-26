cmake_minimum_required(VERSION 3.5)
project(bdrive_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-implicit-fallthrough)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 添加电机驱动库
list(APPEND CMAKE_PREFIX_PATH "/usr/local/bzl_robot/lib/cmake/motor-driver") 
find_package(motor-driver REQUIRED) 
include_directories(${motor-driver_INCLUDE_DIRS})
link_libraries(${motor-driver_LIBRARIES})

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/bdrive_hardware_interface.cpp
  src/bdrive_helper.cpp
)

pluginlib_export_plugin_description_file(hardware_interface bdrive_hardware_interface.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME} 
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib )

ament_auto_package()
