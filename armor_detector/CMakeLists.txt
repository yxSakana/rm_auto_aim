cmake_minimum_required(VERSION 3.8)
project(armor_detector)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARN 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#########
# Build #
#########
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("-- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME}
    Threads::Threads
    Eigen3::Eigen
    fmt::fmt
    ${OpenCV_LIBS}
    openvino::runtime
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "armor_auto_aim::ArmorDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    model
    launch
)
