cmake_minimum_required(VERSION 3.14)

project(
  VisualServoingPlugin
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(ament_cmake)
find_package(geometry_msgs REQUIRED)

# Install the plugin configuration
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/VisualServoingPlugin.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/VisualServoingPlugin.cpp)

set(plugin_HDR src/VisualServoingPlugin.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(VisualServoingPlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(VisualServoingPlugin PUBLIC mc_rtc::mc_rtc_ros)
ament_target_dependencies(VisualServoingPlugin PUBLIC geometry_msgs)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
