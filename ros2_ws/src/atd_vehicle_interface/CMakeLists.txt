cmake_minimum_required(VERSION 3.16)
project(atd_vehicle_interface)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)
ament_auto_add_library(atd_vehicle_interface_component SHARED src/pico_node.cpp)
target_compile_features(atd_vehicle_interface_component PUBLIC cxx_std_20)
target_compile_options(atd_vehicle_interface_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(atd_vehicle_interface_component yaml-cpp)
rclcpp_components_register_node(atd_vehicle_interface_component
  PLUGIN "atd_vehicle_interface::PicoNode" EXECUTABLE atd_vehicle_interface_node)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
# 小さい編集元だけを同梱する。Pico SDK と ARM 成果物は ROS の build/install に含めない。
install(DIRECTORY
  firmware/pico_rev6/src firmware/pico_rev6/include
  firmware/pico_rev6/scripts firmware/pico_rev6/tests
  DESTINATION share/${PROJECT_NAME}/firmware/pico_rev6
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.c" PATTERN "*.h" PATTERN "*.sh" PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE PATTERN ".cache" EXCLUDE
  PATTERN ".git" EXCLUDE PATTERN "vendor" EXCLUDE PATTERN "build" EXCLUDE)
install(FILES firmware/pico_rev6/CMakeLists.txt firmware/pico_rev6/README.md
  firmware/pico_rev6/BENCH_CHECK.md
  DESTINATION share/${PROJECT_NAME}/firmware/pico_rev6)
if(BUILD_TESTING)
  add_executable(test_pico_protocol test/test_protocol.cpp)
  target_compile_features(test_pico_protocol PRIVATE cxx_std_20)
  target_include_directories(test_pico_protocol PRIVATE include)
  add_test(NAME pico_protocol COMMAND test_pico_protocol)
  add_executable(test_pico_serial test/test_serial.cpp)
  target_compile_features(test_pico_serial PRIVATE cxx_std_20)
  target_include_directories(test_pico_serial PRIVATE include)
  add_test(NAME pico_serial COMMAND test_pico_serial)
  add_test(NAME pico_firmware_host COMMAND "${CMAKE_COMMAND}" -E env
    "CC=${CMAKE_C_COMPILER}"
    "JPBB_PICO_TEST_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/firmware-host-tests"
    "PYTHONDONTWRITEBYTECODE=1"
    bash "${CMAKE_CURRENT_SOURCE_DIR}/firmware/pico_rev6/scripts/test.sh")
endif()
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
