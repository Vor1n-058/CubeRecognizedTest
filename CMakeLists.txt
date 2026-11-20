cmake_minimum_required(VERSION 3.8)
project(yjt_recognized_cube)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV 4.9.0 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 编译并注册自定义服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectCube.srv"
)

# 3. 添加主程序可执行文件
add_executable(recognizedCube src/recognizedCube.cpp)

target_include_directories(recognizedCube PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(recognizedCube
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

# 4. 添加控制台客户端可执行文件
add_executable(cubeClient src/cubeClient.cpp)

target_include_directories(cubeClient PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cubeClient
  rclcpp
)

# 5. 链接自动生成的服务接口库代码和 OpenCV
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(recognizedCube ${OpenCV_LIBS} "${cpp_typesupport_target}")
target_link_libraries(cubeClient "${cpp_typesupport_target}")

# 6. 安装节点可执行文件与样图资源到 install 目录
install(TARGETS recognizedCube cubeClient DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()