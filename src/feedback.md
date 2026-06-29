# 问题反馈

本文档整理 Team15 项目在 Airbox Q900 上复现过程中遇到的问题、原因分析、处理方式和当前状态。

## 1. Airbox 默认 ROS2 Jazzy，项目需要 ROS1 Noetic

**现象**

Airbox 当前系统环境是 ROS2 Jazzy，而 Team15 项目代码依赖 ROS1 Noetic、catkin、rospy、actionlib、MoveIt 等 ROS1 组件，不能直接在 ROS2 环境中运行。

**原因**

Team15 原项目是 ROS1 包，包含 `.launch`、`catkin_make`、`rospy`、`actionlib`、`moveit_commander` 等 ROS1 工作流。ROS2 Jazzy 与 ROS1 Noetic 的包结构、通信接口和构建系统不兼容。

**处理方式**

没有卸载系统 ROS2，而是使用 Docker 创建独立 ROS1 Noetic 容器：

```bash
sudo docker run -it --name team15-noetic \
  --platform linux/arm64 \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /home/radxa/Team15:/home/radxa/Team15 \
  ros:noetic-ros-base-focal bash
```

如果无法直接拉取 Docker Hub 镜像，则在另一台电脑拉取 arm64 镜像并导出 rootfs，再在 Airbox 上 `docker import`。

**当前状态**

已解决。ROS1 Noetic 容器 `team15-noetic` 已可运行。



## 2. `docker save` arm64 镜像失败

**现象**

在另一台电脑执行：

```bash
sudo docker save ros:noetic-ros-base-focal -o noetic-ros-base-focal-arm64.tar
```

出现类似错误：

```text
unable to create manifests file: NotFound: content digest ... not found
```

**原因**

本地 Docker daemon 中的多架构镜像 manifest / layer 内容不完整，导致 `docker save` 无法正确导出镜像。

**处理方式**

改用 `docker create` + `docker export` 导出 rootfs，而不是 `docker save` 导出完整 Docker image。

**当前状态**

已解决。使用 rootfs tar 成功导入 Airbox。



## 3. ROS 依赖缺失

**现象**

构建时陆续出现缺包错误，例如：

```text
Could not find a package configuration file provided by "controller_manager"
Could not find a package configuration file provided by "cv_bridge"
```

`rosdep check` 显示缺少：

```text
ros-noetic-cv-bridge
python3-opencv
ros-noetic-robot-state-publisher
ros-noetic-rviz
ros-noetic-joint-state-publisher-gui
ros-noetic-moveit-ros-move-group
ros-noetic-moveit-fake-controller-manager
ros-noetic-moveit-kinematics
ros-noetic-moveit-planners-ompl
ros-noetic-moveit-ros-visualization
ros-noetic-moveit-setup-assistant
ros-noetic-moveit-simple-controller-manager
ros-noetic-joint-state-publisher
ros-noetic-xacro
ros-noetic-joy
ros-noetic-diagnostic-updater
```

**原因**

基础 Noetic 镜像只包含 ROS core/base，不包含 MoveIt、cv_bridge、usb_cam、smach 等项目运行依赖。

**处理方式**

在容器中安装必要依赖：

```bash
apt update
apt install -y \
  python3-pip \
  python3-opencv \
  v4l-utils \
  libyaml-cpp-dev \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-usb-cam \
  ros-noetic-smach \
  ros-noetic-smach-ros \
  ros-noetic-moveit \
  ros-noetic-moveit-commander \
  ros-noetic-controller-manager \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-rviz \
  ros-noetic-xacro \
  ros-noetic-joy \
  ros-noetic-diagnostic-updater
```

**当前状态**

已解决。`catkin_make` 已成功完成。

## 4. Python 模块缺失：`smach`、`moveit_commander`

**现象**

直接导入主程序测试 LLM parser 时，先后出现：

```text
ModuleNotFoundError: No module named 'smach'
ModuleNotFoundError: No module named 'moveit_commander'
```

**原因**

主程序文件在模块顶层导入了 ROS 状态机和 MoveIt 相关模块，即使只测试 LLM parser，也会触发这些依赖导入。

**处理方式**

安装：

```bash
apt install -y ros-noetic-smach ros-noetic-smach-ros ros-noetic-moveit ros-noetic-moveit-commander
```

**当前状态**

已解决。LLM parser 可单独导入并测试。

## 5. Airbox 本地模型服务适配

**现象**

项目原本更偏向 OpenAI-compatible API，但 Airbox 使用本地 Genie API 服务，需要确认本地模型是否可用，并适配本地接口返回格式。

**原因**

Airbox 本地模型服务地址为：

```text
http://127.0.0.1:8910/v1
```

模型包括：

```text
DeepSeek-R1-Distill-Qwen-7B
Qwen2.0-7B-SSD
Phi-3.5-mini
Llama3.2-3B
```

部分模型可能输出 `<think>` 或 markdown fenced JSON，需要清理后再解析。

**处理方式**

主程序中将默认 LLM 地址改为 Airbox 本地服务，并增强 JSON 清理逻辑：

* 去除 `<think>...</think>`
* 去除 ```json 代码块标记
* 从模型回复中截取 `{...}` JSON 主体
* LLM 失败时回退到本地规则解析

同时新增启动脚本：

```text
scripts/start_airbox_genie_service.sh
```

**当前状态**

已解决。已验证文本 `把红色方块放到B区` 可解析为结构化任务。

## 6. Airbox上无法部署SAM3

**现象**

Airbox上无法部署SAM3进行视觉识别

**原因分析**

官方 SAM3 仓库当前要求 Python 3.12+、PyTorch 2.7+、以及 CUDA-compatible GPU with CUDA 12.6+；官方安装示例也走 CUDA 版 PyTorch。Airbox 没有 NVIDIA CUDA GPU，所以本机部署官方 SAM3 推理不现实。

**处理方式**

改换用YOLO代替SAM3进行视觉识别

**当前状态**

已用YOLO替代SAM3完成视觉识别部分


## 7. 单帧 YOLO-World 检测结果验证

**现象**

需要确认摄像头图像能否进入 YOLO-World 并输出检测框。

**处理方式**

从 `/usb_cam/image_raw` 获取一帧，通过 `cv_bridge` 转成 OpenCV 图像，再调用 YOLO-World。

**验证结果**

成功读取图像：

```text
image shape: (480, 640, 3)
```

成功检测到目标示例：

```text
detections: 1
cup 0.5373 [0.0, 223.6, 31.0, 325.2]
```

**当前状态**

已验证。

## 8. Airbox 无法识别 Type-C 麦克风

**现象**

Type-C 麦克风插入 Airbox 后没有明显反应，系统未检测到可用录音设备。

**原因分析**

可能原因包括：

* 当前 Type-C 口不支持该麦克风的数据/供电模式
* 麦克风需要 USB Audio Class 支持但未被枚举
* 需要 USB-A 转接、OTG 转接或带供电 HUB
* 容器中没有正确映射音频设备

**处理方式**

暂未解决。当前绕过语音输入，先使用 `_task_text:=...` 文本方式验证任务解析和机械臂控制链路。

**当前状态**

未解决。实时语音输入尚未验证。




## 当前总体状态

已完成：

* ROS1 Noetic Docker 环境搭建
* Team15 catkin 工作区构建
* Airbox 本地模型任务解析
* YOLO-World 权重加载
* RealSense 515 彩色流接入
* `/usb_cam/image_raw` 图像 topic 验证
* 单帧目标检测验证
* mock action server dry-run 机制

