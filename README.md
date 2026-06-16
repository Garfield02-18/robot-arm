## Airbox Reproduction Status

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
* Real Sagittarius arm pick and place execution

Airbox-related files included in this package:

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

### Create a ROS1 Noetic Container

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
  -v /home/radxa/Team15:/home/radxa/Team15 \
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

### Build the Team15 Workspace

The `src` directory in this repository is itself the ROS package `sagittarius_object_color_detector`. Inside the container, create a catkin workspace and link this package into it:

```bash
source /opt/ros/noetic/setup.bash

mkdir -p /home/radxa/team15_ws/src
ln -sfn \
  /home/radxa/Team15/Team15-FinalProject/src \
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

### Airbox Local Model

Start the local model service on the Airbox host:

```bash
/home/radxa/Team15/Team15-FinalProject/src/scripts/start_airbox_genie_service.sh
```

Verify it from inside the ROS1 container:

```bash
curl http://127.0.0.1:8910/v1/models
```

The response should include models such as `DeepSeek-R1-Distill-Qwen-7B`.

### Enter the ROS1 Container

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

### YOLO-World Verification

Before running YOLO / torch, set:

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

### Camera Verification

The verified RealSense 515 RGB device is:

```text
/dev/video6
```

Start the image stream:

```bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/video6
```

Verify the image topic from another container terminal:

```bash
rostopic hz /usb_cam/image_raw
```

### Text-Input Dry Run

When no real robot arm is available, use the mock action server to verify that control commands are generated.

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

The mock mode does not move any real object. After `PICK_XYZ` and `PUT_XYZ` appear, the dry run is considered successful.
