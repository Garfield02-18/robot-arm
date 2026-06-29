# Deployment Guide

This repository is the Radxa Airbox adaptation of the voice-guided robotic sorting project. The goal of this version is not only to preserve the original ROS1 sorting pipeline, but also to make the project reproducible and deployable on a Radxa Airbox Q900 as an edge AI robot controller.

In this deployment, the Airbox is the central device of the whole system. It runs the local LLM service, hosts the ROS1 Noetic Docker container, receives images from the RealSense 515 camera, performs object detection, and sends pick/place commands to the Sagittarius robot arm through ROS action messages.

Before starting, please make sure you have read the relevant documentation for the Radxa Airbox Q900 https://docs.radxa.com/fogwise/airbox-q900

## Demo Video

The project demonstration video is available here:

[Watch the demo video](demo/demo_cut.mp4)

If your Markdown viewer supports embedded videos, it can also be played below:

<video src="demo/demo_cut.mp4" controls width="100%">
  Your browser does not support embedded video. Please use the link above to watch the demo.
</video>

## Why the Deployment Is Built Around Radxa Airbox

The original project depends on ROS1 Noetic, MoveIt, camera input, natural-language task parsing, and robot-arm action control. The Airbox provides a compact edge-computing platform where these components can be deployed on one device:

* The Airbox host keeps its original system environment, including ROS2 Jazzy if it is already installed.
* A Docker container provides an isolated ROS1 Noetic runtime for the Team15 ROS package.
* The Airbox local model service provides an OpenAI-compatible API at `http://127.0.0.1:8910/v1`.
* USB devices such as the RealSense camera and robot-arm serial devices are passed into the container through `/dev`.
* ROS nodes, MoveIt, YOLO-World, HSV detection, and action clients run inside the ROS1 container.
* The local model service runs on the Airbox host, and the container accesses it through host networking.

This design avoids replacing the system ROS environment on the Airbox and makes the ROS1 project easier to reproduce on different Airbox devices.

## Airbox-Centered Runtime Architecture

| Layer | Runs On | Responsibility |
| --- | --- | --- |
| Airbox host OS | Radxa Airbox Q900 | Docker daemon, USB devices, local model service, QAI/Genie environment |
| Local model API | Airbox host | Converts natural-language instructions into structured task JSON |
| ROS1 Noetic container | Docker on Airbox | Runs the catkin workspace, ROS nodes, MoveIt, YOLO-World, HSV detection, and action clients |
| RealSense 515 | USB device on Airbox | Provides the RGB image stream, verified as `/dev/video6` |
| Sagittarius robot arm | Connected through Airbox device interfaces | Receives `SGRCtrl` pick/place action commands |
| Mock action server | ROS1 container | Used for dry-run verification when the real robot arm is unavailable |

Important runtime detail: the local LLM is served by the Airbox local model stack, while YOLO-World currently runs through Ultralytics/PyTorch inside the ROS1 container. In this version, YOLO-World uses CPU inference, not the Airbox NPU. HSV detection and MoveIt also run on CPU.

## Verified Pipeline on Airbox Q900

The following pipeline has been verified on the Airbox Q900:

```text
Text task
    ↓
Airbox local model / local rule parser
    ↓
Structured task JSON
    ↓
RealSense 515 RGB image
    ↓
YOLO-World / HSV object detection
    ↓
Target robot-arm coordinates
    ↓
SGRCtrl action pick / place command
```

Verified items:

* ROS1 Noetic Docker container: `team15-noetic`
* Recommended reproduction workspace: `/home/radxa/team15_ws`
* Airbox local model API: `http://127.0.0.1:8910/v1`
* Text task parsing, for example `put the red block into area B` / `把红色方块放到B区`
* YOLO-World weight loading: `models/yolov8s-worldv2.pt`
* RealSense 515 RGB stream: `/dev/video6`
* ROS image topic: `/usb_cam/image_raw`, about `16.6 Hz`
* Single-frame YOLO-World detection
* Robot-free dry run with `nodes/mock_sgr_ctrl_server.py`

Not yet verified:

* Live microphone input: the current Airbox does not detect the microphone device
* Real Sagittarius robot-arm pick and place execution

## Airbox Deployment Path Assumptions

The commands below assume the following paths on the Airbox:

```text
/home/radxa/robot_arm/src
/home/radxa/team15_ws
/home/radxa/ai-engine-direct-helper
/home/radxa/qairt
/home/radxa/miniconda3/envs/llm
```

If another Airbox uses a different username or project path, update the Docker volume mount and the workspace symlink accordingly.

The ROS1 container is launched with these key options:

* `--network host`: allows ROS nodes inside the container to access the Airbox host local model API through `127.0.0.1:8910`.
* `--privileged` and `-v /dev:/dev`: make camera and robot hardware devices visible inside the container.
* `-v /home/radxa/robot_arm:/home/radxa/robot_arm`: shares the project files between the Airbox host and the container.
* `-v /tmp/.X11-unix:/tmp/.X11-unix`: allows GUI tools such as RViz to use the host display when available.

## Airbox-Related Files Included in This Package

```text
src/
├── CMakeLists.txt
├── package.xml
├── action/
│   └── SGRCtrl.action
├── config/
│   ├── HSVParams.cfg
│   └── vision_config.yaml
├── launch/
│   ├── llm_safe_sort_demo_planningscene_api_voice.launch
│   └── usb_cam.launch
├── models/
│   └── yolov8s-worldv2.pt
├── nodes/
│   ├── llm_safe_sort_demo_planningscene_api_voice.py
│   ├── mock_sgr_ctrl_server.py
│   ├── sgr_ctrl.py
│   └── voice_task_input.py
└── scripts/
    └── start_airbox_genie_service.sh
```

## Create a ROS1 Noetic Container on Airbox

The Airbox system may already have ROS2 Jazzy installed. Do not remove the system ROS2 installation. Use Docker to create an isolated ROS1 Noetic environment instead.

If the Airbox can access Docker Hub directly:

```bash
sudo docker pull --platform linux/arm64 ros:noetic-ros-base-focal
```

If Docker Hub access is unstable on the Airbox, prepare an arm64 rootfs on another Linux machine with working network access:

```bash
sudo docker pull --platform linux/arm64 ros:noetic-ros-base-focal
sudo docker create \
  --platform linux/arm64 \
  --name noetic-rootfs \
  ros:noetic-ros-base-focal \
  /bin/bash

sudo docker export noetic-rootfs -o noetic-ros-base-focal-rootfs-arm64.tar
sudo docker rm noetic-rootfs
```

Copy `noetic-ros-base-focal-rootfs-arm64.tar` to the Airbox, then import it on the Airbox:

```bash
sudo docker import \
  /home/radxa/noetic-ros-base-focal-rootfs-arm64.tar \
  ros:noetic-ros-base-focal-imported
```

Create the ROS1 container for Team15:

```bash
sudo docker run -it --name team15-noetic \
  --platform linux/arm64 \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /home/radxa/robot_arm:/home/radxa/robot_arm \
  ros:noetic-ros-base-focal bash
```

If you are using the offline imported image, replace the last image line with:

```bash
ros:noetic-ros-base-focal-imported bash
```

Install dependencies inside the container on the first run:

```bash
apt update

apt install -y \
  git \
  python3-pip \
  python3-opencv \
  v4l-utils \
  libyaml-cpp-dev \
  ros-noetic-catkin \
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

Install the YOLO-World runtime dependency:

```bash
python3 -m pip install ultralytics==8.3.0 \
  -i https://mirrors.aliyun.com/pypi/simple \
  --trusted-host mirrors.aliyun.com \
  --timeout 600 --retries 20 \
  --progress-bar off
```

## Build the Team15 Workspace Inside the Airbox Container

The `src` directory in this repository is itself the ROS package `sagittarius_object_color_detector`. After entering the container, create a catkin workspace and link this package into it:

```bash
source /opt/ros/noetic/setup.bash

mkdir -p /home/radxa/team15_ws/src
ln -sfn \
  /home/radxa/robot_arm/src \
  /home/radxa/team15_ws/src/sagittarius_object_color_detector

cd /home/radxa/team15_ws
catkin_make
source devel/setup.bash
```

Verify that ROS can find the package:

```bash
rospack find sagittarius_object_color_detector
```

Expected output:

```text
/home/radxa/team15_ws/src/sagittarius_object_color_detector
```

## Start the Airbox Local Model Service

The local model service should be started on the Airbox host, not inside the ROS1 container. The script uses the Airbox host-side Conda, QAI, and model paths.

```bash
/home/radxa/robot_arm/src/scripts/start_airbox_genie_service.sh
```

Verify the service from inside the ROS1 container:

```bash
curl http://127.0.0.1:8910/v1/models
```

The response should include models such as `DeepSeek-R1-Distill-Qwen-7B`.

Because the Docker container uses `--network host`, ROS nodes can directly call the Airbox host model API through:

```text
http://127.0.0.1:8910/v1
```

## Enter the ROS1 Container

Start and attach to the existing Noetic container:

```bash
sudo docker start -ai team15-noetic
```

From another terminal, enter the running container:

```bash
sudo docker exec -it team15-noetic bash
```

Initialize the ROS environment inside the container:

```bash
source /opt/ros/noetic/setup.bash
cd /home/radxa/team15_ws
source devel/setup.bash
```

## YOLO-World on Airbox

YOLO-World is loaded by the main ROS node from:

```text
models/yolov8s-worldv2.pt
```

The model path is passed through `launch/llm_safe_sort_demo_planningscene_api_voice.launch`:

```xml
<arg name="yolo_model_path"
     default="$(find sagittarius_object_color_detector)/models/yolov8s-worldv2.pt"/>
```

At runtime, the main node reads one frame from `/usb_cam/image_raw`, converts it with `cv_bridge`, and calls Ultralytics/PyTorch inference. This process runs inside the ROS1 container and currently uses CPU inference.

Before running YOLO / torch inside the Airbox container, set:

```bash
export LD_PRELOAD=$(find /usr/local/lib/python3.8/dist-packages -name 'libgomp*.so*' | head -1)
```

Verify that the model can be loaded:

```bash
python3 - <<'PY2'
from ultralytics import YOLOWorld

path = "/home/radxa/team15_ws/src/sagittarius_object_color_detector/models/yolov8s-worldv2.pt"
model = YOLOWorld(path)
model.set_classes(["block", "cube", "red block", "blue block", "green block", "cup", "banana"])
print("YOLO-World model loaded OK")
PY2
```

## Camera Deployment on Airbox

On the tested Airbox, the RealSense 515 RGB device is:

```text
/dev/video6
```

On another Airbox or after changing the camera, confirm the device first:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video6 --list-formats-ext
```

Start the image stream inside the ROS1 container:

```bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/video6
```

From another container terminal, verify the image topic:

```bash
rostopic hz /usb_cam/image_raw
```

On the tested Airbox, `/usb_cam/image_raw` runs at about `16.6 Hz`.

## Text-Input Dry Run on Airbox

When no real robot arm is available, use the mock action server to verify that the Airbox can run the complete software control chain:

```text
Text task -> local model / rule parser -> vision result -> SGRCtrl pick/place action
```

Terminal 1: keep the camera running:

```bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/video6
```

Terminal 2: start the mock robot-arm action server:

```bash
rosrun sagittarius_object_color_detector mock_sgr_ctrl_server.py
```

Terminal 3: start the main program with a text task:

```bash
export LD_PRELOAD=$(find /usr/local/lib/python3.8/dist-packages -name 'libgomp*.so*' | head -1)

rosrun sagittarius_object_color_detector llm_safe_sort_demo_planningscene_api_voice.py \
  _vision_config:=/home/radxa/team15_ws/src/sagittarius_object_color_detector/config/vision_config.yaml \
  _arm_name:=sgr532 \
  _task_text:="把蓝色方块放到B区" \
  _use_llm:=true \
  _llm_url:=http://127.0.0.1:8910/v1 \
  _llm_model:=DeepSeek-R1-Distill-Qwen-7B \
  _yolo_model_path:=/home/radxa/team15_ws/src/sagittarius_object_color_detector/models/yolov8s-worldv2.pt \
  _show_debug:=false
```

If the mock server terminal prints logs similar to the following, the text task has been converted into robot-arm control commands:

```text
[MockSGR] goal action=XYZ_RPY ...
[MockSGR] goal action=PICK_XYZ pos=(...)
[MockSGR] goal action=PUT_XYZ pos=(0.150, 0.240, 0.200)
```

The mock mode does not move any real object. After `PICK_XYZ` and `PUT_XYZ` appear, the dry run can be considered successful.

## Real Robot Deployment Notes

For a real Sagittarius robot-arm deployment, the main launch file starts:

* `sagittarius_moveit/launch/demo_true.launch`
* `sgr_ctrl.py` as the `/sgr532/sgr_ctrl` action server
* `usb_cam.launch` for camera input
* `voice_task_input.py` for voice/text task input
* `llm_safe_sort_demo_planningscene_api_voice.py` for task parsing, object detection, PlanningScene updates, and pick/place orchestration

The current Airbox reproduction has verified the perception and command-generation path. Real robot-arm motion still requires hardware-side validation, including calibration, workspace range, gripper behavior, and safety boundaries.

## Known Limitations

* YOLO-World currently runs on CPU through PyTorch inside the ROS1 container. It has not been converted to the Airbox NPU/QNN runtime.
* The local LLM service depends on Airbox host-side QAI/Genie paths and is therefore Airbox-specific.
* The tested Type-C microphone was not detected by the Airbox, so live voice input has not yet been verified.
* Real Sagittarius robot-arm pick/place execution has not yet been verified in the current Airbox environment.
* On another Airbox, `/dev/video6` may not be the RGB camera device. Always verify it with `v4l2-ctl`.

For a detailed issue log, see `问题反馈.md`.
