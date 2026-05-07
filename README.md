# AIR5021 Team 15 —— 语音引导智能机械臂分拣系统

> 基于 Sagittarius 机械臂平台，实现的语音交互、多模态感知、LLM 任务解析与 MoveIt PlanningScene 避障的一体化智能分拣系统。

---

# 1. 项目简介

本项目基于 `sagittarius_arm_ros` 机械臂平台，实现了一个桌面场景下的语音引导智能分拣系统。

系统支持：

* 语音输入
* 实时录音
* 自然语言任务解析
* 多目标视觉识别
* MoveIt PlanningScene 避障
* 机械臂抓取与放置
* 多任务顺序执行

系统整体流程如下：

```text
语音 / 文本指令
        ↓
Whisper 语音识别
        ↓
任务文本
        ↓
LLM / 本地规则解析
        ↓
结构化任务列表
        ↓
YOLO / HSV 混合视觉识别
        ↓
像素坐标 → 机械臂坐标
        ↓
PlanningScene 更新障碍物
        ↓
Sagittarius 机械臂抓取与放置
```

本项目为 AIR5021 Final Project 最终版本。

---

# 2. 项目目录

项目主要位于：

```text
/home/robotics/team15/src/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/
```

常用目录说明：

```text
sagittarius_object_color_detector/
├── launch/        # ROS launch 文件
├── nodes/         # 可执行 ROS 节点
├── config/        # 配置文件
├── scripts/       # 辅助脚本
├── action/        # 自定义 Action
└── ...
```

---

# 3. 环境要求

推荐环境：

* Ubuntu 20.04
* ROS Noetic
* Python 3
* MoveIt
* 已安装 `sagittarius_arm_ros`
* 已完成 catkin 工作区编译

---

# 4. 工作区初始化

进入工作区：

```bash
cd /home/robotics/team15/src/sagittarius_ws
```

加载 ROS 环境：

```bash
source devel/setup.bash
```

检查 ROS 是否正确指向当前工作区：

```bash
rospack find sagittarius_object_color_detector
```

---

# 5. 依赖安装

---

## 5.1 安装 Whisper

Whisper 用于语音识别：

```bash
pip3 install --user openai-whisper
```

---

## 5.2 安装 OpenCC

Whisper 有时会输出繁体中文，因此使用 OpenCC 做繁简转换：

```bash
pip3 install --user opencc-python-reimplemented
```

---

## 5.3 安装 FFmpeg

Whisper 读取音频依赖 FFmpeg。

检查：

```bash
ffmpeg -version
```

若未安装：

```bash
sudo apt install ffmpeg
```

---

# 6. 编译项目

如果修改了代码：

```bash
cd /home/robotics/team15/src/sagittarius_ws

catkin_make

source devel/setup.bash
```

---

# 7. 启动方式

最终版本 launch 文件：

```text
llm_safe_sort_demo_planningscene_api_voice.launch
```

---

## 7.1 使用音频文件启动

```bash
roslaunch sagittarius_object_color_detector \
llm_safe_sort_demo_planningscene_api_voice.launch \
audio_path:=/home/robotics/task.wav
```

系统会：

1. 读取音频文件
2. 调用 Whisper 识别文本
3. 发布任务文本
4. 执行智能分拣

---

## 7.2 实时录音模式

直接启动：

```bash
roslaunch sagittarius_object_color_detector \
llm_safe_sort_demo_planningscene_api_voice.launch
```

随后根据终端提示进行实时录音。

---

## 7.3 直接输入文本任务

```bash
roslaunch sagittarius_object_color_detector \
llm_safe_sort_demo_planningscene_api_voice.launch \
task_text:="把最左边的蓝色方块放到A区"
```

如果 `task_text` 非空，则优先使用文本任务，不等待语音输入。

---

## 7.4 通过 ROS Topic 输入音频路径

启动系统后：

```bash
rostopic pub /voice_audio_path std_msgs/String \
"data: '/home/robotics/task.wav'"
```

---

# 8. 当前支持的任务类型

---

## 按颜色抓取

```text
把蓝色方块放到A区
```

---

## 按左右位置抓取

```text
把最左边的方块放到B区
```

---

## 颜色 + 方位组合

```text
把最左边的蓝色方块放到A区
```

---

## 多任务顺序执行

```text
把trash can放到B区，然后把cup放到D区
```

---

# 9. 语音识别模块

语音识别节点：

```text
nodes/voice_task_input.py
```

功能包括：

* 音频文件读取
* 实时录音
* Whisper 识别
* OpenCC 繁简转换
* 发布识别结果到：

```text
/voice_task_text
```

如果语音识别结果为空：

* 系统会自动退回终端输入模式
* 用户可手动输入任务文本

---

# 10. LLM 任务解析

系统支持：

* LLM 任务解析
* 本地规则解析 fallback

推荐 LLM 返回格式：

```json
{
  "tasks": [
    {
      "color": "blue",
      "position": "left",
      "place_name": "A"
    }
  ]
}
```

若：

* LLM API 调用失败
* 输出格式错误
* 网络异常

则自动退回本地规则解析。

---

# 11. 视觉识别系统

当前系统使用：

* YOLO 接口
* HSV 颜色识别

其中：

## YOLO

用于：

* 多类别扩展
* 开放词汇目标检测

## HSV

用于：

* 彩色方块稳定识别
* 颜色校验 fallback

当前系统会：

* 使用 YOLO 获取目标位置
* 使用 HSV 验证目标颜色

---

# 12. PlanningScene 避障

抓取前系统会：

* 将桌面加入 PlanningScene
* 将非目标物体加入碰撞场景

作用：

* 提供基础避障能力
* 提高路径规划真实性
* 避免直接穿过障碍物

如果 RViz 中能看到：

* 桌面模型
* 障碍物 collision box

则通常说明 PlanningScene 已成功接入。

---

# 13. 可调参数

---

## 13.1 搜索位姿

重要参数：

```python
goal_search.pos_x
goal_search.pos_y
goal_search.pos_z
goal_search.pos_roll
goal_search.pos_pitch
goal_search.pos_yaw
```

推荐调整流程：

1. 在 RViz 中手动调整机械臂
2. 打印当前末端位姿
3. 复制到搜索位姿函数中

---

## 13.2 工作区范围

代码中通常包含：

```python
def in_workspace(x, y):
```

如果经常出现：

```text
Target out of workspace
```

则需调整：

* `x_min`
* `x_max`
* `y_min`
* `y_max`

---

## 13.3 障碍物尺寸

PlanningScene 中可调整：

* 桌面尺寸
* 障碍物高度
* inflate_scale

若尺寸不合理，可能导致：

* 规划失败
* 避障效果不明显
* 模型与真实物体不一致

---

## 13.4 YOLO 参数

常见参数：

```python
yolo_model_path
conf_thres
show_debug
```

---

# 14. 系统鲁棒性设计

最终版本加入了多个保护机制：

* ASR fallback
* LLM fallback
* 本地规则 fallback
* 抓取失败后重新检测
* 无效目标过滤
* 工作区合法性检查
* 规划失败重试
* 终端手动输入 fallback

这些机制显著提升了真实机械臂上的稳定性。

---

# 15. 常见问题

---

## roslaunch 找不到节点

检查：

```bash
chmod +x your_node.py
```

并确认 launch 中：

```xml
type=
```

与真实文件名一致。

---

## exit code 127

通常原因：

* Windows 换行符
* shebang 错误

修复：

```bash
sed -i 's/\r$//' your_node.py
```

---

## 一直等待 `/get_planning_scene`

通常是 MoveIt namespace 不一致。

例如机械臂 namespace 为：

```text
/sgr532
```

则：

* robot_description
* MoveIt namespace

也必须对应 `/sgr532`。

---

## 规划失败

优先检查：

* 搜索位姿
* 工作区范围
* 标定参数
* 障碍物尺寸

---

# 16. 后续改进方向

---

## 完整 YOLO 部署

当前仍保留 HSV fallback。

后续可：

* 完全替换为 YOLO
* 支持更多类别
* 提高复杂场景鲁棒性

---

## RGB-D 三维避障

当前使用简化 box 模型。

后续可加入：

* RGB-D 相机
* 三维障碍物重建
* 更精确碰撞模型

---

## 连续语音对话

后续可扩展：

* 多轮对话
* 歧义询问
* 持续语音助手

---

## 更完善任务管理

未来可增加：

* 任务队列持久化
* 自动重试机制
* 抓取成功验证
* 动态优先级管理

---

# 17. 项目亮点

本项目主要亮点包括：

* 语音引导机械臂交互
* LLM 自然语言任务解析
* YOLO / HSV 混合感知
* MoveIt PlanningScene 避障
* 多重 fallback 鲁棒性设计
* 真实硬件完整运行
* 模块化 ROS 架构

---

# 18. 成员分工

* **Peng Zhiyuan**
  系统整体集成、PlanningScene、机械臂执行、调试

* **Xu Xuanchen**
  LLM 任务解析与结构化任务表示

* **Liu Yibo**
  YOLO 接口与实时录音功能

* **Zhan Zihao**
  调试、测试与系统支持

---

# 19. 总结

本项目最终实现了一个完整的：

* ASR 语音识别
* LLM 任务解析
* 混合视觉感知
* PlanningScene 避障
* 真实机械臂抓取执行

的一体化语音引导智能分拣系统，并成功运行于真实 Sagittarius 机械臂平台。
