#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// 包含由 DetectCube.srv 编译后生成的服务头文件
#include "yjt_recognized_cube/srv/detect_cube.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
struct Detection
{
  std::string color;
  cv::Rect rect;
  int area;
};

constexpr double kMinContourArea = 3000.0;

std::string trim(const std::string & input)
{
  auto not_space = [](unsigned char c) {return !std::isspace(c);};
  auto begin = std::find_if(input.begin(), input.end(), not_space);
  auto end = std::find_if(input.rbegin(), input.rend(), not_space).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

std::string toLower(std::string input)
{
  std::transform(
    input.begin(), input.end(), input.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return input;
}

cv::Scalar colorToBgr(const std::string & color)
{
  if (color == "red") {
    return cv::Scalar(0, 0, 255);
  }
  if (color == "green") {
    return cv::Scalar(0, 255, 0);
  }
  if (color == "blue") {
    return cv::Scalar(255, 0, 0);
  }
  return cv::Scalar(255, 255, 255);
}
}  // namespace

class RecognizedCubeNode : public rclcpp::Node
{
public:
  RecognizedCubeNode()
  : Node("recognized_cube")
  {
    // 1. 获取样图绝对路径并读取图片
    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("yjt_recognized_cube");
    image_path_ = pkg_share + "/resource/Cube.png";
    raw_image_ = cv::imread(image_path_);

    if (raw_image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "无法读取图片，请检查路径: %s", image_path_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "图片路径: %s", image_path_.c_str());
    }
    display_image_ = raw_image_.clone();

    // 2. 初始化话题发布者：发布压缩图像供 Foxglove 监视
    img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/cube/image/compressed", 10);

    // 3. 初始化服务服务端：创建 /detect_cube 服务并绑定回调函数
    service_ = this->create_service<yjt_recognized_cube::srv::DetectCube>(
      "/detect_cube",
      std::bind(&RecognizedCubeNode::handleDetectCube, this, _1, _2));

    // 4. 定时循环发布图像（每隔 100ms 刷新一次 Foxglove 画面）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RecognizedCubeNode::publishImage, this));
  }

private:
  // 服务回调函数：按目标颜色识别方块并返回坐标
  void handleDetectCube(
    const std::shared_ptr<yjt_recognized_cube::srv::DetectCube::Request> request,
    std::shared_ptr<yjt_recognized_cube::srv::DetectCube::Response> response)
  {
    const std::string color = toLower(trim(request->target_color));
    RCLCPP_INFO(this->get_logger(), "收到识别请求，目标颜色: %s", color.c_str());

    if (raw_image_.empty()) {
      response->success = false;
      response->message = "样图读取失败";
      return;
    }

    if (color != "red" && color != "green" && color != "blue") {
      RCLCPP_WARN(this->get_logger(), "未知颜色请求: %s", color.c_str());
      response->success = false;
      response->message = "未知颜色: " + color + "（可选 red / green / blue）";
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(raw_image_, hsv, cv::COLOR_BGR2HSV);
    const auto detections = detectSingleColor(hsv, color);

    updateDisplay(detections);
    fillResponse(detections, color, *response);
  }

  // 对单一颜色做 HSV 阈值分割，返回所有有效方块
  std::vector<Detection> detectSingleColor(
    const cv::Mat & hsv, const std::string & color) const
  {
    std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;

    if (color == "red") {
      // 红色在 HSV 色环两端，需要两段阈值
      ranges = {
        {cv::Scalar(0, 100, 70), cv::Scalar(10, 255, 255)},
        {cv::Scalar(170, 100, 70), cv::Scalar(180, 255, 255)}};
    } else if (color == "blue") {
      ranges = {{cv::Scalar(95, 80, 60), cv::Scalar(130, 255, 255)}};
    } else if (color == "green") {
      ranges = {{cv::Scalar(35, 80, 70), cv::Scalar(85, 255, 255)}};
    } else {
      return {};
    }

    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto & range : ranges) {
      cv::Mat current;
      cv::inRange(hsv, range.first, range.second, current);
      cv::bitwise_or(mask, current, mask);
    }

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Detection> detections;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < kMinContourArea) {
        continue;
      }
      detections.push_back({color, cv::boundingRect(contour), static_cast<int>(area)});
    }
    return detections;
  }

  // 在图像上绘制识别框和颜色标签
  void updateDisplay(const std::vector<Detection> & detections)
  {
    display_image_ = raw_image_.clone();

    for (const auto & detection : detections) {
      const auto color = colorToBgr(detection.color);
      cv::rectangle(display_image_, detection.rect, color, 3);

      const int center_x = detection.rect.x + detection.rect.width / 2;
      const int center_y = detection.rect.y + detection.rect.height / 2;
      const std::string label =
        detection.color + " (" + std::to_string(center_x) + ", " +
        std::to_string(center_y) + ")";
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(
        label, cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseline);
      const cv::Point origin(
        detection.rect.x,
        std::max(detection.rect.y - 10, text_size.height + 5));
      cv::putText(
        display_image_, label, origin, cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
    }
  }

  // 填充服务响应
  void fillResponse(
    const std::vector<Detection> & detections,
    const std::string & target,
    yjt_recognized_cube::srv::DetectCube::Response & response) const
  {
    response.success = !detections.empty();
    response.count = static_cast<int32_t>(detections.size());

    for (const auto & detection : detections) {
      response.colors.push_back(detection.color);
      response.centers_x.push_back(detection.rect.x + detection.rect.width / 2);
      response.centers_y.push_back(detection.rect.y + detection.rect.height / 2);
      response.widths.push_back(detection.rect.width);
      response.heights.push_back(detection.rect.height);
      response.areas.push_back(detection.area);

      RCLCPP_INFO(
        this->get_logger(), "识别到 %s 方块: x:%d y:%d w:%d h:%d area:%d",
        detection.color.c_str(), detection.rect.x, detection.rect.y,
        detection.rect.width, detection.rect.height, detection.area);
    }

    if (detections.empty()) {
      response.message = "未识别到颜色为 " + target + " 的方块";
      RCLCPP_WARN(this->get_logger(), "%s", response.message.c_str());
    } else {
      response.message =
        "识别到 " + std::to_string(detections.size()) + " 个颜色为 " + target + " 的方块";
      RCLCPP_INFO(this->get_logger(), "%s", response.message.c_str());
    }
  }

  // 定时器回调：将带标注框的图像转化为压缩图像并发布
  void publishImage()
  {
    if (display_image_.empty()) {
      return;
    }

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = this->now();
    cv_img.header.frame_id = "cube_image";
    cv_img.encoding = "bgr8";
    cv_img.image = display_image_;

    const auto msg = cv_img.toCompressedImageMsg(cv_bridge::Format::JPEG);
    img_pub_->publish(*msg);
  }

  std::string image_path_;
  cv::Mat raw_image_;
  cv::Mat display_image_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr img_pub_;
  rclcpp::Service<yjt_recognized_cube::srv::DetectCube>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RecognizedCubeNode>());
  rclcpp::shutdown();
  return 0;
}