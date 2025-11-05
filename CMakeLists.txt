cmake_minimum_required(VERSION 3.15)

project(
  RobotModelUpdate
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/RobotModelUpdate.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/RobotModelUpdate.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RobotModelUpdate.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_HDR src/RobotModelUpdate.h src/Convex.h)

set(AUTOLOAD_RobotModelUpdate_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load ${PLUGIN_NAME} plugin")
# The add_plugin macro is provided by mc_rtc
set(RobotModelUpdatePluginName "RobotModelUpdate")
configure_file(src/RobotModelUpdate.in.cpp
               "${CMAKE_CURRENT_BINARY_DIR}/src/RobotModelUpdate.cpp")
set(plugin_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/RobotModelUpdate.cpp")
add_plugin(RobotModelUpdate "${plugin_SRC}" "${plugin_HDR}")
target_include_directories(
  RobotModelUpdate PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

# FIXME: mc_rtc currently does not support loading multiple plugins with the
# same name! For now create a second identical version of the plugin with a
# different name
set(RobotModelUpdatePluginName "RobotModelUpdate2")
configure_file(src/RobotModelUpdate.in.cpp
               "${CMAKE_CURRENT_BINARY_DIR}/src/RobotModelUpdate2.cpp")
set(plugin_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/RobotModelUpdate2.cpp")
add_plugin(RobotModelUpdate2 "${plugin_SRC}" "${plugin_HDR}")
target_include_directories(
  RobotModelUpdate2 PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RobotModelUpdate.yaml"
  DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc"
  RENAME "RobotModelUpdate2.yaml")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
