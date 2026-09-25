cmake_minimum_required(VERSION 3.5)
project(uav_rl_guidance)

# C++17 (required for ONNX Runtime)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# GeographicLib
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(px4_msgs    REQUIRED)
find_package(uav_common_msg REQUIRED)
find_package(GeographicLib  REQUIRED)
find_package(Eigen3         REQUIRED)

# ONNX Runtime
find_library(ONNXRUNTIME_LIB onnxruntime PATHS /usr/local/lib REQUIRED)
include_directories(/usr/local/include)
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIB}")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(uav_rl_guidance
  src/rl_guidance_node.cpp
)

target_link_libraries(uav_rl_guidance
  ${ONNXRUNTIME_LIB}
  ${GeographicLib_LIBRARIES}
)

ament_target_dependencies(uav_rl_guidance
  rclcpp
  px4_msgs
  uav_common_msg
)

install(TARGETS uav_rl_guidance
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES config/params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
