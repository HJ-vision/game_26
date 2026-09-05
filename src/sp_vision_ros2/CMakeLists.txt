cmake_minimum_required(VERSION 3.16)
project(sp_vision_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 依赖查找
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenMP REQUIRED)

# OpenVINO：需要先安装，并在环境变量或下方设置安装路径
# 例：export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# ============================================================
# 同济算法核心（与 sp_vision_25 原始 CMake 对齐，去掉了
# multithread / planner / auto_buff / omniperception .cpp）
# ============================================================
set(AUTO_AIM_SRCS
  tasks/auto_aim/armor.cpp
  tasks/auto_aim/classifier.cpp
  tasks/auto_aim/detector.cpp
  tasks/auto_aim/solver.cpp
  tasks/auto_aim/aimer.cpp
  tasks/auto_aim/target.cpp
  tasks/auto_aim/tracker.cpp
  tasks/auto_aim/yolo.cpp
  tasks/auto_aim/yolos/yolov5.cpp
  tasks/auto_aim/yolos/yolov8.cpp
  tasks/auto_aim/yolos/yolo11.cpp
)

# ============================================================
# TensorRT（可选，仅 Jetson 上存在）
# 四件套全找到（NvInfer.h + libnvinfer + cuda_runtime_api.h + libcudart）
# 才编译 yolov5_trt.cpp 并定义 SP_VISION_HAS_TENSORRT；
# 找不到则编纯 CPU/OpenVINO 版本，yolo.cpp 的 device=GPU 分发
# 运行时回退并打 WARN。Jetson 需要 libnvinfer-dev + cuda-toolkit。
#
# 【09-04 实测教训】TRT 头依赖的是 cuda_runtime_api.h
# （NvInferRuntimeBase.h:24 include 它），不是 cuda_runtime.h——
# 探测文件必须与实际依赖对齐，否则会出现"探测通过但编译失败"。
# HINTS 覆盖 JetPack 三种布局：/usr/local/cuda（symlink）、
# /usr/local/cuda-12.6（无 symlink）、deb targets 目录。
# ============================================================
find_path(TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS /usr/include /usr/local/cuda/include /usr/include/aarch64-linux-gnu)
find_library(TENSORRT_NVINFER_LIBRARY
  NAMES nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
        /usr/local/cuda/lib64 /usr/lib)
find_path(CUDA_RUNTIME_INCLUDE_DIR
  NAMES cuda_runtime_api.h
  HINTS /usr/local/cuda/include
        /usr/local/cuda-12.6/include
        /usr/local/cuda/targets/aarch64-linux/include
        /usr/local/cuda-12.6/targets/aarch64-linux/include
        /usr/include)
find_library(CUDA_RUNTIME_LIBRARY
  NAMES cudart
  HINTS /usr/local/cuda/lib64
        /usr/local/cuda-12.6/lib64
        /usr/local/cuda/targets/aarch64-linux/lib
        /usr/local/cuda-12.6/targets/aarch64-linux/lib
        /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib)

if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIBRARY AND
   CUDA_RUNTIME_INCLUDE_DIR AND CUDA_RUNTIME_LIBRARY)
  set(SP_VISION_HAS_TENSORRT ON)
  # 探测结果全路径打印（在 log/latest_build/.../stdout.log 里查）
  message(STATUS "sp_vision_ros2: TensorRT found -> yolov5 GPU backend ENABLED")
  message(STATUS "  NvInfer.h : ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "  nvinfer   : ${TENSORRT_NVINFER_LIBRARY}")
  message(STATUS "  cuda hdr  : ${CUDA_RUNTIME_INCLUDE_DIR}")
  message(STATUS "  cudart    : ${CUDA_RUNTIME_LIBRARY}")
  list(APPEND AUTO_AIM_SRCS tasks/auto_aim/yolos/yolov5_trt.cpp)
else()
  message(STATUS
    "sp_vision_ros2: TensorRT NOT found -> CPU-only (OpenVINO) backends")
  message(STATUS "  NvInfer.h : ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "  nvinfer   : ${TENSORRT_NVINFER_LIBRARY}")
  message(STATUS "  cuda hdr  : ${CUDA_RUNTIME_INCLUDE_DIR}")
  message(STATUS "  cudart    : ${CUDA_RUNTIME_LIBRARY}")
endif()

set(TOOLS_SRCS
  tools/exiter.cpp
  tools/extended_kalman_filter.cpp
  tools/ransac_sine_fitter.cpp
  tools/img_tools.cpp
  tools/math_tools.cpp
  tools/trajectory.cpp
  tools/logger.cpp
  tools/pid.cpp
  tools/crc.cpp
)

add_library(sp_vision_core OBJECT ${AUTO_AIM_SRCS} ${TOOLS_SRCS})
target_include_directories(sp_vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_compile_features(sp_vision_core PUBLIC cxx_std_17)
target_link_libraries(sp_vision_core
  Eigen3::Eigen
  fmt::fmt
  spdlog::spdlog
  yaml-cpp
  OpenMP::OpenMP_CXX
  openvino::runtime
)
target_link_libraries(sp_vision_core
  ${OpenCV_LIBS}
)
ament_target_dependencies(sp_vision_core
  rclcpp
  sensor_msgs
  cv_bridge
  auto_aim_interfaces
)

# ---- TensorRT 接线（仅探测成功时）----
# 【09-04 踩坑】target_link_libraries 必须用 plain 签名（无 PUBLIC 关键字）：
# sp_vision_core 的既有调用（Eigen3/OpenCV）和 ament_target_dependencies
# 内部全是 plain，CMake 禁止同一 target 混用 plain/keyword 两种签名。
# plain 对 OBJECT 库的传播与既有 Eigen3 完全一致（符号进 vision_node 链接行）。
# include/compile_definitions 不受此限制，PUBLIC 照常（自身编译+传播消费者）。
if(SP_VISION_HAS_TENSORRT)
  target_include_directories(sp_vision_core PUBLIC
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_RUNTIME_INCLUDE_DIR}
  )
  target_compile_definitions(sp_vision_core PUBLIC SP_VISION_HAS_TENSORRT=1)
  target_link_libraries(sp_vision_core
    ${TENSORRT_NVINFER_LIBRARY}
    ${CUDA_RUNTIME_LIBRARY}
  )
endif()

# ============================================================
# 视觉 ROS2 节点
# ============================================================
add_executable(vision_node src/vision_node.cpp)
target_include_directories(vision_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_compile_features(vision_node PRIVATE cxx_std_17)
target_link_libraries(vision_node
  sp_vision_core
  Eigen3::Eigen
  fmt::fmt
  spdlog::spdlog
  yaml-cpp
  OpenMP::OpenMP_CXX
  openvino::runtime
  ${OpenCV_LIBS}
)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  auto_aim_interfaces
)

install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# 模型与配置随包安装（运行时可从 install/share 下拷贝到工作目录，
# 或直接给 config_path 传绝对路径）
install(DIRECTORY assets configs
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
