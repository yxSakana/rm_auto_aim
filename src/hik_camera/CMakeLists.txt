cmake_minimum_required(VERSION 3.8)
project(hik_camera)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARN 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Threads REQUIRED)
find_package(glog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC /opt/MVS/include)

target_link_libraries(${PROJECT_NAME}
    Threads::Threads
    fmt::fmt
    glog::glog
    ${OpenCV_LIBS}
    MvCameraControl
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hik_camera::HikCameraNode"
  EXECUTABLE hik_camera_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
