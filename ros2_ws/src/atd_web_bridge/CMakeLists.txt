cmake_minimum_required(VERSION 3.16)
project(atd_web_bridge)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/web_bridge scripts/extract_drive_timeline scripts/diagnose_stereo_bag DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bag_drive_timeline test/test_bag_drive_timeline.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_calibration)
  ament_add_pytest_test(stereo_bag_diagnostics test/test_stereo_bag_diagnostics.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_calibration)
  ament_add_pytest_test(recording_launch test/test_recording_launch.py)
  ament_add_pytest_test(auto_drive_monitor test/test_auto_drive_window.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(gazebo_monitor test/test_gazebo_bridge.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
  ament_add_pytest_test(bridge_localization test/test_localization.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
  ament_add_pytest_test(bridge_core test/test_core.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
  ament_add_pytest_test(teleop_routes test/test_teleop_routes.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
  ament_add_pytest_test(propo_routes test/test_propo_routes.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
  ament_add_pytest_test(vehicle_routes test/test_vehicle_routes.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
  ament_add_pytest_test(recording_routes test/test_recording_routes.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../atd_joy:${CMAKE_CURRENT_SOURCE_DIR}/../atd_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../atd_rc_input:${CMAKE_CURRENT_SOURCE_DIR}/../atd_vehicle_description)
endif()
ament_auto_package()
