#include "yolov5_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

// ============================================================================
// TrtLogger：spdlog -> nvinfer1::ILogger 桥接
// ============================================================================
void TrtLogger::log(Severity severity, const char * msg) noexcept
{
  if (!msg) return;
  switch (severity) {
    case Severity::kINTERNAL_ERROR:
    case Severity::kERROR:
      tools::logger()->error("[TRT] {}", msg);
      break;
    case Severity::kWARNING:
      tools::logger()->warn("[TRT] {}", msg);
      break;
    case Severity::kINFO:
      // TRT 的 info 很多，降为 debug 只写文件，避免刷终端
      tools::logger()->debug("[TRT] {}", msg);
      break;
    default:
      break;
  }
}

// ============================================================================
// 构造 / 析构
// ============================================================================
YOLOV5TRT::YOLOV5TRT(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  // TRT 专用路径（缺键兜底，不与 OpenVINO 的 yolov5_model_path 混用）
  onnx_path_ = yaml["yolov5_trt_onnx_path"]
                 ? yaml["yolov5_trt_onnx_path"].as<std::string>()
                 : "assets/0526.onnx";
  engine_path_ = yaml["yolov5_trt_engine_path"]
                   ? yaml["yolov5_trt_engine_path"].as<std::string>()
                   : "assets/yolov5_fp16.engine";
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  input_size_ = yaml["input_size"] ? yaml["input_size"].as<int>() : 640;
  if (input_size_ % 32 != 0) {
    tools::logger()->warn("[YOLOV5TRT] input_size={} 不是 32 的倍数，向上取整", input_size_);
    input_size_ = ((input_size_ + 31) / 32) * 32;
  }
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  if (!load_or_build_engine()) {
    throw std::runtime_error(
      "[YOLOV5TRT] failed to load or build TensorRT engine from " + onnx_path_);
  }
}

YOLOV5TRT::~YOLOV5TRT()
{
  if (stream_) cudaStreamDestroy(stream_);
  if (dev_input_) cudaFree(dev_input_);
  if (dev_output_) cudaFree(dev_output_);
  if (host_input_) delete[] host_input_;
  if (host_output_) delete[] host_output_;
  // TRT 10：destroy() 已被移除（8.5 弃用 → 10.0 删除），统一用 delete。
  // 删除顺序：先下游 context，再 engine，最后 runtime。
  if (context_) {
    delete context_;
    context_ = nullptr;
  }
  if (engine_) {
    delete engine_;
    engine_ = nullptr;
  }
  if (runtime_) {
    delete runtime_;
    runtime_ = nullptr;
  }
}

// ============================================================================
// engine 生命周期：缓存加载 / trtexec 构建 / 反序列化
// ============================================================================
bool YOLOV5TRT::load_or_build_engine()
{
  namespace fs = std::filesystem;

  auto onnx_exists = fs::exists(onnx_path_);
  auto engine_exists = fs::exists(engine_path_);
  bool need_build = !engine_exists;

  // engine 比 onnx 旧 → 模型更新过，重建
  if (onnx_exists && engine_exists) {
    auto onnx_mtime = fs::last_write_time(onnx_path_);
    auto engine_mtime = fs::last_write_time(engine_path_);
    if (engine_mtime < onnx_mtime) {
      tools::logger()->warn(
        "[YOLOV5TRT] engine older than onnx, rebuilding: {}", engine_path_);
      need_build = true;
    }
  }

  if (need_build) {
    if (!onnx_exists) {
      tools::logger()->error(
        "[YOLOV5TRT] neither engine nor onnx found: '{}' / '{}'", engine_path_, onnx_path_);
      return false;
    }
    if (!build_engine_with_trtexec()) return false;
  }

  if (!deserialize_engine(engine_path_)) {
    // 反序列化失败（TRT 版本不匹配 / 换板 / 文件损坏）→ 尝试重建一次
    tools::logger()->warn(
      "[YOLOV5TRT] deserialize failed, try rebuilding from onnx: {}", onnx_path_);
    if (!onnx_exists || !build_engine_with_trtexec() || !deserialize_engine(engine_path_)) {
      return false;
    }
  }
  return true;
}

bool YOLOV5TRT::build_engine_with_trtexec()
{
  // trtexec 位置：libnvinfer-bin 装在 /usr/src/tensorrt/bin/（不在默认 PATH）
  std::string trtexec = "/usr/src/tensorrt/bin/trtexec";
  if (!std::filesystem::exists(trtexec)) {
    // 备选：PATH 里找
    if (std::system("which trtexec >/dev/null 2>&1") == 0) {
      trtexec = "trtexec";
    } else {
      tools::logger()->error(
        "[YOLOV5TRT] trtexec not found at {} nor in PATH (install libnvinfer-bin)", trtexec);
      return false;
    }
  }

  std::string cmd = fmt::format(
    "{} --onnx={} --fp16 --saveEngine={} 2>&1 | tail -5", trtexec, onnx_path_, engine_path_);
  tools::logger()->info(
    "[YOLOV5TRT] building fp16 engine (takes 1~3 min on Jetson): {} --onnx={} --fp16 "
    "--saveEngine={}",
    trtexec, onnx_path_, engine_path_);
  int rc = std::system(cmd.c_str());
  if (rc != 0 || !std::filesystem::exists(engine_path_)) {
    tools::logger()->error("[YOLOV5TRT] trtexec build failed (rc={})", rc);
    return false;
  }
  tools::logger()->info("[YOLOV5TRT] engine built: {}", engine_path_);
  return true;
}

bool YOLOV5TRT::deserialize_engine(const std::string & path)
{
  namespace fs = std::filesystem;
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) return false;
  auto size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  if (!file.read(data.data(), size)) return false;
  file.close();

  if (!runtime_) runtime_ = nvinfer1::createInferRuntime(trt_logger_);
  engine_ = runtime_->deserializeCudaEngine(data.data(), size);
  if (!engine_) return false;
  context_ = engine_->createExecutionContext();
  if (!context_) return false;

  // ---- I/O 张量自动发现（TRT10 tensor-name API，不硬编码名字）----
  const int n_io = engine_->getNbIOTensors();
  for (int i = 0; i < n_io; i++) {
    const char * name = engine_->getIOTensorName(i);
    auto mode = engine_->getTensorIOMode(name);
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      input_name_ = name;
    } else {
      output_name_ = name;
    }
  }
  if (input_name_.empty() || output_name_.empty()) {
    tools::logger()->error(
      "[YOLOV5TRT] failed to discover I/O tensor names (in='{}', out='{}')", input_name_,
      output_name_);
    return false;
  }

  // ---- 输入 shape 校验 + 分配 ----
  auto in_dims = engine_->getTensorShape(input_name_.c_str());
  if (in_dims.nbDims != 4 || in_dims.d[0] != 1 || in_dims.d[1] != 3) {
    tools::logger()->error(
      "[YOLOV5TRT] unexpected input dims: {}x{}x{}x{} (expect 1x3xHxW)", in_dims.d[0],
      in_dims.d[1], in_dims.d[2], in_dims.d[3]);
    return false;
  }
  trt_input_size_ = static_cast<int>(in_dims.d[2]);
  if (in_dims.d[3] != in_dims.d[2]) {
    tools::logger()->warn(
      "[YOLOV5TRT] non-square input {}x{}, using {}", in_dims.d[2], in_dims.d[3], trt_input_size_);
  }
  if (trt_input_size_ != input_size_) {
    tools::logger()->warn(
      "[YOLOV5TRT] yaml input_size={} != engine {} —— 以 engine 为准（重新导出 onnx "
      "或改 yaml 可消除此告警）",
      input_size_, trt_input_size_);
    input_size_ = trt_input_size_;
  }
  const size_t in_elems = 1ull * 3 * trt_input_size_ * trt_input_size_;
  host_input_ = new float[in_elems];
  if (cudaMalloc(&dev_input_, in_elems * sizeof(float)) != cudaSuccess) return false;

  // ---- 输出 shape 校验（【关键】通道数必须 22，错配直接失败）----
  auto out_dims = engine_->getTensorShape(output_name_.c_str());
  if (out_dims.nbDims != 3 || out_dims.d[0] != 1) {
    tools::logger()->error(
      "[YOLOV5TRT] unexpected output rank: {} (expect 3)", out_dims.nbDims);
    return false;
  }
  // 兼容 [1, N, C] 与 [1, C, N] 两种布局：通道数是 22 的那维
  long d1 = out_dims.d[1], d2 = out_dims.d[2];
  if (d1 == kOutputChannels) {
    output_n_ = static_cast<int>(d2);  // [1, 22, N]（少见）
    output_c_ = static_cast<int>(d1);
    tools::logger()->warn(
      "[YOLOV5TRT] output layout [1,{},{}] is channel-first —— 需要转置读取", d1, d2);
    return false;  // 0526 是 [1,N,22]；channel-first 属于意外布局，先不支持
  } else if (d2 == kOutputChannels) {
    output_n_ = static_cast<int>(d1);  // [1, N, 22]（0526 实测布局）
    output_c_ = static_cast<int>(d2);
  } else {
    // 【上次 best2-sim 崩溃教训】通道错配是内存越界，必须 throw 而非 warn
    tools::logger()->error(
      "[YOLOV5TRT] output channels mismatch: [{},{},{}] —— 期望末维或次维 = {} "
      "(8 kpt + 1 obj + 4 color + 9 class)。这不是 0526 RM 模型！",
      out_dims.d[0], d1, d2, kOutputChannels);
    return false;
  }
  const size_t out_elems = 1ull * output_n_ * kOutputChannels;
  host_output_ = new float[out_elems];
  if (cudaMalloc(&dev_output_, out_elems * sizeof(float)) != cudaSuccess) return false;

  if (cudaStreamCreate(&stream_) != cudaSuccess) return false;

  tools::logger()->info(
    "[YOLOV5TRT] engine ready: in='{}' [1,3,{},{}] out='{}' [1,{},{}] ({} anchors x {} "
    "ch)",
    input_name_, trt_input_size_, trt_input_size_, output_name_, output_n_, output_c_,
    output_n_, output_c_);
  return true;
}

// ============================================================================
// 推理
// ============================================================================
std::list<Armor> YOLOV5TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }
  if (!context_) {
    tools::logger()->error("[YOLOV5TRT] no context, skip frame");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  // ---- 预处理（与 OpenVINO 版几何一致：等比缩放 + letterbox 黑边）----
  const int S = trt_input_size_;
  auto x_scale = static_cast<double>(S) / bgr_img.rows;
  auto y_scale = static_cast<double>(S) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  canvas_ = cv::Mat(S, S, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, canvas_(roi), {w, h});

  // blobFromImage: BGR→RGB(swapRB) + /255(scale) + HWC→CHW + u8→f32
  cv::Mat blob;
  cv::dnn::blobFromImage(canvas_, blob, 1.0 / 255.0, cv::Size(S, S), cv::Scalar(), true, false, CV_32F);
  std::memcpy(host_input_, blob.ptr<float>(), 1ull * 3 * S * S * sizeof(float));

  // ---- 推理（H2D → enqueueV3 → D2H，同一 stream 保证顺序）----
  // 【TRT10 关键】enqueueV3 前必须 setInputShape（即使静态 shape）
  bool ok_shape = context_->setInputShape(input_name_.c_str(), nvinfer1::Dims4{1, 3, S, S});
  bool ok_addr_in = context_->setTensorAddress(input_name_.c_str(), dev_input_);
  bool ok_addr_out = context_->setTensorAddress(output_name_.c_str(), dev_output_);
  cudaError_t h2d_err = cudaMemcpyAsync(dev_input_, host_input_,
                      1ull * 3 * S * S * sizeof(float),
                      cudaMemcpyHostToDevice, stream_);
  // 诊断：前 5 帧验证输入数据链路（setShape/绑定/H2D/回读）
  static int diag_h2d = 0;
  if (diag_h2d < 5) {
    diag_h2d++;
    float hmin = 1e9f, hmax = -1e9f;
    bool hnan = false;
    for (size_t i = 0; i < (size_t)(3 * S * S); i++) {
      float v = host_input_[i];
      if (std::isnan(v) || std::isinf(v)) hnan = true;
      if (v < hmin) hmin = v;
      if (v > hmax) hmax = v;
    }
    tools::logger()->warn("[TRT-DIAG] ok_shape={} addr(in={},out={}) h2d={}",
                          ok_shape, ok_addr_in, ok_addr_out, (int)h2d_err);
    tools::logger()->warn(
      "[TRT-DIAG] host_in[0..9]: {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} "
      "{:.3f} {:.3f} {:.3f} | min={:.3f} max={:.3f} nan={}",
      host_input_[0], host_input_[1], host_input_[2], host_input_[3], host_input_[4],
      host_input_[5], host_input_[6], host_input_[7], host_input_[8], host_input_[9], hmin,
      hmax, hnan);
    // 回读 dev_input 验证 H2D 生效（Jetson 统一内存需确认拷贝真的写了）
    cudaStreamSynchronize(stream_);
    float dp[10];
    cudaMemcpy(dp, dev_input_, 10 * sizeof(float), cudaMemcpyDeviceToHost);
    tools::logger()->warn(
      "[TRT-DIAG] dev_in[0..9]: {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} "
      "{:.3f} {:.3f} {:.3f}",
      dp[0], dp[1], dp[2], dp[3], dp[4], dp[5], dp[6], dp[7], dp[8], dp[9]);
  }
  if (h2d_err != cudaSuccess) {
    tools::logger()->error("[YOLOV5TRT] H2D memcpy failed");
    return std::list<Armor>();
  }
  // 诊断：推理前清零 dev_output_。推理后看 host_output_：
  //   全 0 → 推理没写 dev_output_（绑定问题）
  //   NaN → 推理写了 NaN（推理本身问题）
  //   正常值 → 推理正常（则 D2H 链路有问题）
  cudaMemsetAsync(dev_output_, 0,
                  1ull * output_n_ * kOutputChannels * sizeof(float), stream_);
  // 【改用 enqueueV2】enqueueV3+setTensorAddress 在本环境（Jetson+TRT10.3）
  // 虽返回 true 但输出未写入（NaN）。enqueueV2 用 bindings 数组直接传地址，
  // TRT 8 经典写法、TRT 10 仍支持（deprecated 但稳定），trtexec 也用它。
  void * bindings[8] = {};
  for (int i = 0; i < engine_->getNbIOTensors(); i++) {
    const char * name = engine_->getIOTensorName(i);
    auto mode = engine_->getTensorIOMode(name);
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      bindings[i] = dev_input_;
    } else {
      bindings[i] = dev_output_;
    }
  }
  if (!context_->enqueueV2(bindings, stream_, nullptr)) {
    tools::logger()->error("[YOLOV5TRT] enqueueV2 failed");
    return std::list<Armor>();
  }
  if (cudaMemcpyAsync(host_output_, dev_output_,
                      1ull * output_n_ * kOutputChannels * sizeof(float),
                      cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
    tools::logger()->error("[YOLOV5TRT] D2H memcpy failed");
    return std::list<Armor>();
  }
  // enqueueV3 内部的 CUDA 错误只在 sync 时暴露（返回 true 不代表推理成功）
  cudaError_t sync_err = cudaStreamSynchronize(stream_);
  if (sync_err != cudaSuccess) {
    tools::logger()->error("[YOLOV5TRT] stream sync failed: {} (enqueue 内部可能出错)",
                           cudaGetErrorString(sync_err));
    cudaGetLastError();  // 清除错误状态，避免污染后续帧
    return std::list<Armor>();
  }

  // ---- 临时诊断：前 5 帧打印输出摘要，定位"无框"根因 ----
  static int diag_count = 0;
  if (diag_count < 5) {
    diag_count++;
    // 全局最大 objectness（通道 8）
    float max_obj = -1e9f;
    int max_idx = 0;
    for (int i = 0; i < output_n_; i++) {
      float v = host_output_[i * kOutputChannels + 8];
      if (v > max_obj) {
        max_obj = v;
        max_idx = i;
      }
    }
    tools::logger()->warn(
      "[TRT-DIAG] frame={} anchors={} max_obj={:.4f} at#{} sigmoid={:.4f} "
      "thr={}",
      frame_count, output_n_, max_obj, max_idx, sigmoid(max_obj),
      score_threshold_);
    // 打印最大 obj anchor 的全部 22 通道（看角点/obj/color/class 分布）
    std::string ch = "[TRT-DIAG] best anchor: ";
    for (int c = 0; c < kOutputChannels; c++) {
      ch += fmt::format("{:.2f} ", host_output_[max_idx * kOutputChannels + c]);
    }
    tools::logger()->warn(ch);
    // 前 3 个 anchor 的通道 0-9（看是否全 0）
    for (int a = 0; a < 3; a++) {
      std::string s = fmt::format("[TRT-DIAG] a{}: ", a);
      for (int c = 0; c < 10; c++) {
        s += fmt::format("{:.2f} ", host_output_[a * kOutputChannels + c]);
      }
      tools::logger()->warn(s);
    }
  }

  // ---- 后处理：[N, 22] Mat，与 OpenVINO 版 parse 逐位一致 ----
  cv::Mat output(output_n_, kOutputChannels, CV_32F, host_output_);
  return parse(scale, output, raw_img, frame_count);
}

// ============================================================================
// 后处理：与 YOLOV5（OpenVINO 版）完全一致，只是数据源从 OV tensor 换成 TRT 缓冲
// ============================================================================
std::list<Armor> YOLOV5TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

std::list<Armor> YOLOV5TRT::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: 8 kpt + obj + color + class
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    // 颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     // color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  // num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < (int)armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5TRT::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;
  return name_ok && confidence_ok;
}

bool YOLOV5TRT::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);
  return name_ok;
}

cv::Point2f YOLOV5TRT::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5TRT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {} (TRT)", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("detection", detection);
}

void YOLOV5TRT::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5TRT::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

}  // namespace auto_aim
