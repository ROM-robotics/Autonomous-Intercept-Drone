cmake_minimum_required(VERSION 3.5)
project(uav_vision_png)

# C99 / C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# GeographicLib（用于统计模块坐标转换，不参与导引算法）
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs    REQUIRED)
find_package(uav_common_msg REQUIRED)
find_package(GeographicLib  REQUIRED)
find_package(Eigen3         REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ---- 可执行目标 ----
add_executable(uav_vision_png
  src/vision_png_control.cpp
)

target_include_directories(uav_vision_png PUBLIC
  ${GeographicLib_INCLUDE_DIRS}
)

target_link_libraries(uav_vision_png
  ${GeographicLib_LIBRARIES}
)

ament_target_dependencies(uav_vision_png
  rclcpp
  std_msgs
  sensor_msgs
  px4_msgs
  uav_common_msg
)

# ---- 安装 ----
install(TARGETS uav_vision_png
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES config/params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
