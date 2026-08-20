cmake_minimum_required(VERSION 3.8)
project(brutusbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

#------------------------------------------------------------------------------
# brutusbot_interface
#------------------------------------------------------------------------------

add_library(brutusbot_interface SHARED
  src/brutusbot_interface.cpp
  src/serial_protocol.cpp)

target_include_directories(brutusbot_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS})

ament_target_dependencies(brutusbot_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib)

target_link_libraries(brutusbot_interface
  ${LIBSERIAL_LIBRARIES})

target_compile_options(brutusbot_interface PRIVATE
  ${LIBSERIAL_CFLAGS_OTHER})

pluginlib_export_plugin_description_file(hardware_interface brutusbot_interface.xml)

install(TARGETS brutusbot_interface
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#------------------------------------------------------------------------------
# test_serial
#------------------------------------------------------------------------------

add_executable(test_serial
  src/test_serial.cpp)

target_include_directories(test_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${LIBSERIAL_INCLUDE_DIRS})

ament_target_dependencies(test_serial
  rclcpp)

target_link_libraries(test_serial
  ${LIBSERIAL_LIBRARIES})

target_compile_options(test_serial PRIVATE
  ${LIBSERIAL_CFLAGS_OTHER})

#------------------------------------------------------------------------------
# test_serial_protocol
#------------------------------------------------------------------------------

add_executable(test_serial_protocol
  src/test_serial_protocol.cpp
  src/serial_protocol.cpp)

target_include_directories(test_serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS})

ament_target_dependencies(test_serial_protocol
  rclcpp)

target_link_libraries(test_serial_protocol
  ${LIBSERIAL_LIBRARIES})

target_compile_options(test_serial_protocol PRIVATE
  ${LIBSERIAL_CFLAGS_OTHER})


install(TARGETS
  test_serial test_serial_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  brutusbot_interface
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
