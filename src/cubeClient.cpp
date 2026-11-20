#include <rclcpp/rclcpp.hpp>

#include "yjt_recognized_cube/srv/detect_cube.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

namespace
{
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

std::string normalizeColor(std::string color)
{
  color = toLower(trim(color));

  if (color == "红" || color == "红色") {
    return "red";
  }
  if (color == "绿" || color == "绿色") {
    return "green";
  }
  if (color == "蓝" || color == "蓝色") {
    return "blue";
  }
  return color;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cube_client");

  auto client = node->create_client<yjt_recognized_cube::srv::DetectCube>("/detect_cube");

  std::cout << "等待识别服务 /detect_cube ..." << std::endl;
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "等待服务时被中断");
      rclcpp::shutdown();
      return 1;
    }
    std::cout << "服务尚未启动，继续等待..." << std::endl;
  }
  std::cout << "服务已连接。" << std::endl;
  std::cout << "请输入要识别的颜色: red / green / blue（输入 q 退出）" << std::endl;

  while (rclcpp::ok()) {
    std::cout << "> " << std::flush;

    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }

    const std::string input = toLower(trim(line));
    if (input == "q" || input == "quit" || input == "exit") {
      break;
    }
    if (input.empty()) {
      continue;
    }

    auto request = std::make_shared<yjt_recognized_cube::srv::DetectCube::Request>();
    request->target_color = normalizeColor(input);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "调用识别服务失败");
      break;
    }

    const auto response = future.get();
    std::cout << "是否成功: " << (response->success ? "是" : "否") << std::endl;
    std::cout << "结果信息: " << response->message << std::endl;
    std::cout << "方块数量: " << response->count << std::endl;

    const size_t count = std::min(
      static_cast<size_t>(response->count),
      std::min(
        response->colors.size(),
        std::min(
          response->centers_x.size(),
          std::min(
            response->centers_y.size(),
            std::min(
              response->widths.size(),
              std::min(response->heights.size(), response->areas.size()))))));

    for (size_t i = 0; i < count; ++i) {
      std::cout << "  [" << i + 1 << "] " << response->colors[i]
                << "  中心=(" << response->centers_x[i] << ", " << response->centers_y[i]
                << ")  宽高=" << response->widths[i] << "x" << response->heights[i]
                << "  面积=" << response->areas[i] << std::endl;
    }
  }

  rclcpp::shutdown();
  return 0;
}