cmake_minimum_required(VERSION 3.16)
project(atd_joy)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake_python REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EVDEV REQUIRED IMPORTED_TARGET libevdev)
find_package(yaml-cpp REQUIRED)

ament_auto_add_library(atd_joy_component SHARED src/joy_node.cpp)
target_compile_features(atd_joy_component PUBLIC cxx_std_20)
target_compile_options(atd_joy_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(atd_joy_component PkgConfig::EVDEV yaml-cpp)
rclcpp_components_register_node(atd_joy_component
  PLUGIN "atd_joy::JoyNode" EXECUTABLE atd_joy_node)

ament_python_install_package(atd_joy_tools)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_mapping test/test_mapping.cpp)
  target_compile_features(test_mapping PRIVATE cxx_std_20)
  target_include_directories(test_mapping PRIVATE include)
  add_test(NAME mapping_core COMMAND test_mapping)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(profile_tools test/test_profile.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
