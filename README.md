# yjt_recognized_cube

基于话题通信与服务通信的颜色方块识别，并在 Foxglove 中可视化。

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- OpenCV 4.9.0
- 依赖包：

```bash
sudo apt install ros-humble-cv-bridge ros-humble-ament-index-cpp ros-humble-foxglove-bridge
```

## 构建

把本包放到工作空间的 `src` 目录下：

```bash
mkdir -p ~/my_ws/src
git clone https://github.com/Vor1n-058/CubeRecognizedTest.git \
  ~/my_ws/src/yjt_recognized_cube

cd ~/my_ws
colcon build
source install/setup.bash
```

## 启动节点

新开一个终端时，先执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/my_ws
source install/setup.bash
```

**终端 1：识别服务端**

```bash
ros2 run yjt_recognized_cube recognizedCube
```

**终端 2：控制台客户端**

```bash
ros2 run yjt_recognized_cube cubeClient
```

然后输入要识别的颜色：

```text
red / green / blue
```

也支持中文：`红`、`绿`、`蓝`。输入 `q` 退出。

**终端 3：Foxglove 桥接**

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Foxglove 可视化

1. 打开 Foxglove
2. Open connection → Foxglove WebSocket
3. 地址填 `ws://localhost:8765`
4. Add panel → Image
5. 话题选择 `/cube/image/compressed`

在客户端输入颜色后，图像中会显示对应颜色的方块标注框。

## 话题与服务

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| 服务 | `/detect_cube` | 请求颜色并返回方块坐标、宽高、面积 |
| 话题 | `/cube/image/compressed` | 带标注的 JPEG 图像，供 Foxglove 显示 |

服务类型：`yjt_recognized_cube/srv/DetectCube`