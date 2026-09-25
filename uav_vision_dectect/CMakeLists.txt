cmake_minimum_required(VERSION 3.5)
project(uav_vision_dectect)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required for ONNX Runtime)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uav_common_msg REQUIRED)
find_package(OpenMP REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ONNX Runtime
find_library(ONNXRUNTIME_LIB onnxruntime PATHS /usr/local/lib REQUIRED)
find_library(ONNXRUNTIME_CUDA_LIB onnxruntime_providers_cuda PATHS /usr/local/lib)
find_library(ONNXRUNTIME_SHARED_LIB onnxruntime_providers_shared PATHS /usr/local/lib)
include_directories(/usr/local/include)
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIB}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(
        ./include/uav_vision_dectect/
        ./include/light_track
)

add_executable(uav_vision_dectect
        ./src/main.cpp
        ./src/uav_topic_subscrib.cpp
        ./src/uav_control.cpp
        ./src/LightTrack.cpp
        src/yolo_detector.cpp
)

target_link_libraries(uav_vision_dectect
        ${OpenCV_LIBS}
        ${OpenMP_CXX_LIBRARIES}
        ${ONNXRUNTIME_LIB}
        ${ONNXRUNTIME_CUDA_LIB}
        ${ONNXRUNTIME_SHARED_LIB}
)

#添加节点的依赖
ament_target_dependencies(uav_vision_dectect
    rclcpp
    cv_bridge
    sensor_msgs
    OpenCV
    px4_msgs
    uav_common_msg
)

install(TARGETS
        uav_vision_dectect
        DESTINATION lib/${PROJECT_NAME}
)

