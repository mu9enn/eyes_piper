# 🍎 Piper Fruit Picking Robot

![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) ![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg) ![Status](https://img.shields.io/badge/Status-Pass-blue.svg)

This project integrates **YOLOv5 object detection**, **MoveIt-based robotic arm control**, and **3D vision localization** to achieve autonomous detection and picking of ripe fruits. Designed for testing on a PC running Ubuntu 20.04 with ROS Noetic, it leverages the **Orbbec Dabai camera** for perception and the **Piper robotic arm** for manipulation.

> ⭐ **Note**: To deploy YOLOv5 detection on an RK3588 board, refer to [airockchip/yolov5 README_rkopt.md](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md) for model conversion and optimization.

---

## 🧭 Quick Start

### 🔧 Build the Workspace

```bash
cd eyes_piper/piper_ros
catkin_make
```

### Install Udev Rules

```bash
cd eyes_piper/piper_ros
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo udevadm trigger
```

### Install Python Dependencies

```bash
pip install -r requirements.txt
```

---

### 🖥️ Launch Workflow (5 Terminals)

#### **Terminal 1: Start Camera and YOLO Detection**

```bash
cd eyes_piper/piper_ros
source devel/setup.bash
roslaunch astra_camera dabai.launch
```

> **Publishes**: Detected fruit coordinates to the `/yolo/label_xyz` topic.

---

#### **Terminal 2: Initialize CAN and Enable Robotic Arm**

```bash
cd eyes_piper/piper_ros
source devel/setup.bash
bash can_activate.sh can0 1000000
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

---

#### **Terminal 3: Start MoveIt Control Service (Optional RViz)**

```bash
cd eyes_piper/piper_ros
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

---

#### **Terminal 4: Publish Static TF (camera_link to gripper_base) & Load YOLO**

```bash
cd eyes_piper/piper_ros
source devel/setup.bash
roslaunch astra_camera cam_function.launch
```

Set parameters to enable object detection and image display:

```bash
source devel/setup.bash
rosparam set /go_detect true
rosparam set /yolo/show_image true
```

---

#### **Terminal 5: Execute Localization and Picking Task (Single Tree)**

```bash
cd eyes_piper/piper_ros
source devel/setup.bash
rosrun moveit_ctrl pick_one.py
```

---

## 🎯 Features

- ✅ **Fruit Detection and 3D Localization**: YOLOv5 detects ripe fruits, and depth maps provide spatial coordinates.
- ✅ **Target Pose Tracking**: Automatically adjusts the arm’s base joint to align the gripper with the target.
- ✅ **MoveIt Precision Control**: Uses the `JointMoveitCtrl` service for accurate end-effector positioning and grasping.
- ✅ **Gripper Control**: Opens the gripper before approaching and closes it precisely at the target.

---

## 📦 Project Structure (piper_ros/src)

```bash
├── ros_astra_camera/                # ROS package for Orbbec Dabai camera
│   ├── scripts/
│   │   ├── ros_detect.py           # ROS service script for YOLO detection ⭐
├── piper_moveit/
│   ├── moveit_ctrl/                # MoveIt control services and task execution scripts
│   │   ├── scripts/
│   │   │   ├── pick_one.py        # Script for fruit picking tasks ⭐
│   ├── piper_with_gripper_moveit/
│   │   ├── launch/
│   │   │   ├── static_tf.launch   # Static TF transform between camera_link and gripper_base ⭐
│   │   │   └── demo.launch        # MoveIt controller launch file for Piper arm
├── can_activate.sh                  # Script to initialize CAN device
└── README.md
```

---

## 🔧 Setup Instructions

### Prerequisites
- **Hardware**: Orbbec Dabai camera, Piper robotic arm, PC with Ubuntu 20.04.
- **Software**:
  - ROS Noetic ([Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu))
  - Python 3 and dependencies (listed in `requirements.txt`)
  - Orbbec Astra camera driver ([orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera))
  - Piper ROS SDK ([agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros))

### Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/mu9enn/eyes_piper.git
   cd eyes_piper/piper_ros
   ```
2. **Build the Workspace**:
   ```bash
   catkin_make
   source devel/setup.bash
   ```
3. **Install Udev Rules**:
   Follow the instructions in the Quick Start section to set up camera permissions.
4. **Install Python Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
5. **Configure YOLOv5**:
   - Ensure a trained `.pt` model is available in `piper_ros/src/ros_astra_camera/scripts/` and referenced in `ros_detect.py`.
   - For RK3588 deployment, convert the model to `.rknn` format ([airockchip/yolov5 README_rkopt.md](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md)).

---

## 📚 Usage Notes
- **TF Calibration**: Verify `static_tf.launch` parameters for accurate `camera_link` to `gripper_base` transforms to prevent localization errors.
- **YOLO Model**: Update `ros_detect.py` with the path to your trained `.pt` model or `.rknn` model for RK3588.
- **Testing**: Launch nodes incrementally to ensure each component (camera, YOLO, arm) functions correctly before running `pick_one.py`.
- **Visualization**: Set `/yolo/show_image` to `true` for real-time detection visualization in an OpenCV window.

---

## 🔗 References

- **Orbbec Camera Driver**: [orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera)
- **Piper Robotic Arm API**: [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros)
- **Hand-Eye Calibration**: [IFL-CAMP/easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
- **YOLOv5 for RK3588**: [airockchip/yolov5 README_rkopt.md](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md)

---

## 🙏 Acknowledgments

- **Orbbec**: For the [Astra camera ROS driver](https://github.com/orbbec/ros_astra_camera).
- **AgileX Robotics**: For the [Piper ROS SDK](https://github.com/agilexrobotics/piper_ros).
- **Ultralytics**: For the [YOLOv5 framework](https://github.com/ultralytics/yolov5).
- **IFL-CAMP**: For the [easy_handeye calibration tool](https://github.com/IFL-CAMP/easy_handeye).
