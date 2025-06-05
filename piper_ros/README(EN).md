

# Piper Fruit Picking Robot

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) ![ros](https://img.shields.io/badge/ROS-noetic-blue.svg) ![status](https://img.shields.io/badge/Pass-blue.svg)

> This project integrates **YOLO object detection**, **MoveIt-based robotic arm control**, and **3D vision localization** to automatically identify and pick ripe fruits.

---

## 🧭 Quick Start

### 🔧 Build the Workspace

```bash
cd ~/code_proj/piper_ros
catkin_make
```

---

### 🖥️ Launch Workflow (5 Terminals)

#### **Terminal 1: Start Camera and YOLO Detection**

```bash
source devel/setup.bash
roslaunch astra_camera dabai.launch
```

Enable detection and visualization:

```bash
source devel/setup.bash
rosparam set /go_detect true
rosparam set /yolo/show_image true
```

> Detected object coordinates are published to `/yolo/label_xyz`

---

#### **Terminal 2: Initialize CAN and Enable Arm**

```bash
source devel/setup.bash
bash can_activate.sh can0 1000000
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

---

#### **Terminal 3: Launch MoveIt Control (optional RViz)**

```bash
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

---

#### **Terminal 4: Publish Static TF Between camera\_link and gripper\_base**

```bash
source devel/setup.bash
roslaunch piper_with_gripper_moveit static_tf.launch
```

---

#### **Terminal 5: Execute Localization & Picking (Single Fruit)**

```bash
source devel/setup.bash
rosrun moveit_ctrl pick_one.py
```

---

## 🎯 Feature Overview

* ✅ **Fruit Detection & 3D Localization**: YOLO detects ripe fruits; depth map recovers spatial coordinates.
* ✅ **Base Joint Alignment**: Automatically aligns the gripper base to face the target before picking.
* ✅ **MoveIt Precision Control**: Uses `JointMoveitCtrl` service for pose-aware end-effector motion planning.
* ✅ **Gripper Control**: Opens before grasping, closes tighter once approaching the object.

---

## 📦 Project Structure Overview

```bash
├── astra_camera/             # Camera drivers & depth image publisher
├── moveit_ctrl/              # MoveIt service handlers & pick scripts
├── piper/                    # Low-level arm driver and launcher
├── piper_with_gripper_moveit/
│   ├── launch/
│   │   ├── static_tf.launch     # Static transform between camera_link and gripper_base
│   │   └── demo.launch          # MoveIt control launcher
├── can_activate.sh           # Script to initialize CAN device
└── README.md
```

---

## 🔗 References

* **Orbbec Astra Camera Drivers**
  [https://github.com/orbbec/ros\_astra\_camera](https://github.com/orbbec/ros_astra_camera)

* **AgileX Piper Arm SDK**
  [https://github.com/agilexrobotics/piper\_ros](https://github.com/agilexrobotics/piper_ros)

* **Easy Hand-Eye Calibration**
  [https://github.com/IFL-CAMP/easy\_handeye](https://github.com/IFL-CAMP/easy_handeye)

