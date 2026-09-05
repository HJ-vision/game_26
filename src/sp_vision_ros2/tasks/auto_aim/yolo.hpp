#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  // 【09-04 修复】派生类经 unique_ptr<YOLOBase> 多态删除，基类必须虚析构。
  // 原代码缺此声明，YOLOV5TRT 的 ~override() 编译失败；其他后端虽未写
  // override 也存在 UB 隐患（unique_ptr<基类> 删派生对象），一并消除。
  virtual ~YOLOBase() = default;

  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP