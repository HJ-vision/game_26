#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"
#if defined(SP_VISION_HAS_TENSORRT)
#include "yolos/yolov5_trt.hpp"
#endif

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  // device 键缺省按 CPU 处理（兼容未写该键的旧配置）
  auto device = yaml["device"] ? yaml["device"].as<std::string>() : std::string("CPU");

  // ---- TensorRT GPU 分发（Jetson 专用）----
  // 仅 yolov5 有 TRT 后端（0526.onnx）；其余模型落到下面的 OpenVINO 分支
  //（Jetson 上 OpenVINO 无 GPU plugin，会直接抛异常提示）。
  if (device == "GPU" && yolo_name == "yolov5") {
#if defined(SP_VISION_HAS_TENSORRT)
    yolo_ = std::make_unique<YOLOV5TRT>(config_path, debug);
    return;
#else
    tools::logger()->warn(
      "[YOLO] device=GPU 但本包编译时未找到 TensorRT（CMake 探测 NvInfer.h / "
      "libnvinfer 失败）。回退 OpenVINO CPU 推理。在 Jetson 上装好 "
      "libnvinfer-dev / nvidia-jetpack 后重新 colcon build 即可启用 GPU。");
#endif
  }

  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }

  else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }

  else if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }

  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
