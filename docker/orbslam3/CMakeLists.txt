cmake_minimum_required(VERSION 3.16)
project(ATDOrbslam3 LANGUAGES C CXX)

# This file replaces the top-level build of the pinned checkout in install.sh.
# The tracking library runs headless. RealSense input is provided by the ROS driver.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(APPLE)
    set(CMAKE_INSTALL_RPATH "@loader_path")
else()
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d calib3d imgcodecs)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
get_target_property(G2O_EIGEN3_INCLUDE Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)
target_link_libraries(DBoW2 Boost::serialization)
target_link_libraries(g2o Eigen3::Eigen)
set_target_properties(DBoW2 PROPERTIES OUTPUT_NAME atd_orb_dbow2)
set_target_properties(g2o PROPERTIES OUTPUT_NAME atd_orb_g2o)

file(GLOB sources CONFIGURE_DEPENDS src/*.cc src/*.cpp src/CameraModels/*.cpp)
list(REMOVE_ITEM sources "${PROJECT_SOURCE_DIR}/src/MapDrawer.cc" "${PROJECT_SOURCE_DIR}/src/Viewer.cc")
add_library(core SHARED ${sources})
set_target_properties(core PROPERTIES OUTPUT_NAME atd_orbslam3)
target_compile_definitions(core PRIVATE COMPILEDWITHC11)
target_include_directories(core PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/CameraModels>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/Sophus>
    $<INSTALL_INTERFACE:include/atd_orbslam3>
    $<INSTALL_INTERFACE:include/atd_orbslam3/include>
    $<INSTALL_INTERFACE:include/atd_orbslam3/include/CameraModels>
    $<INSTALL_INTERFACE:include/atd_orbslam3/Thirdparty/Sophus>)
target_link_libraries(core PUBLIC Eigen3::Eigen ${OpenCV_LIBS} Boost::serialization Threads::Threads
                          PRIVATE DBoW2 g2o OpenSSL::Crypto)
install(TARGETS core DBoW2 g2o EXPORT ATDOrbslam3Targets LIBRARY DESTINATION lib)
install(EXPORT ATDOrbslam3Targets NAMESPACE ATDOrbslam3:: DESTINATION lib/cmake/ATDOrbslam3)
install(FILES ATDOrbslam3Config.cmake DESTINATION lib/cmake/ATDOrbslam3)
install(DIRECTORY include/ DESTINATION include/atd_orbslam3/include FILES_MATCHING PATTERN "*.h")
foreach(dep DBoW2/DBoW2 DBoW2/DUtils g2o/g2o Sophus/sophus)
    install(DIRECTORY Thirdparty/${dep}/ DESTINATION include/atd_orbslam3/Thirdparty/${dep}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx")
endforeach()
install(FILES Thirdparty/g2o/config.h DESTINATION include/atd_orbslam3/Thirdparty/g2o)
install(FILES Vocabulary/ORBvoc.txt LICENSE Dependencies.md DESTINATION share/atd_orbslam3)
install(FILES Thirdparty/g2o/license-bsd.txt DESTINATION share/atd_orbslam3/licenses/g2o)
install(FILES Thirdparty/Sophus/LICENSE.txt DESTINATION share/atd_orbslam3/licenses/Sophus)
