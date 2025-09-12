# 🍎 Fruit Picking System on RK3588: YOLO and Piper Robotic Arm

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-RK3588-orange.svg)](https://www.rockchip.com/en/product/rk3588)
[![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://wiki.ros.org/noetic)

**Author**: Xiangyu Sun  
**Created**: July 22, 2025  
**System Environment**: Ubuntu 20.04 on RK3588

---

## 📖 Overview
This project implements an autonomous fruit-picking system on the **RK3588** platform, integrating **YOLOv5** for real-time object detection with the **Orbbec Astra Dabai camera** and the **Piper robotic arm** for precise manipulation. The system leverages the RK3588's Neural Processing Unit (NPU) for efficient YOLO inference and ROS (Robot Operating System) for seamless coordination between perception and control. The project is designed for applications in precision agriculture, such as automated fruit harvesting, and supports deployment on embedded systems like the OrangePi.

Key features:
- **Object Detection**: Uses YOLOv5 for detecting fruits, with model conversion and optimization for RK3588 NPU.
- **3D Localization**: Combines RGB and depth data from the Orbbec camera to compute fruit positions in 3D space.
- **Robotic Manipulation**: Controls the Piper robotic arm via MoveIt! for accurate fruit picking and placement.
- **Integrated Workflow**: A single launch script orchestrates camera, arm, TF tree, and detection/picking nodes.

---

## 📂 Project Structure

```bash
├── piper_ros/                # ROS workspace for Piper robotic arm control
├── vision_ros/               # ROS workspace for camera and YOLO detection
│   ├── src/
│   │   ├── astra_camera/    # Orbbec Astra camera ROS package
│   │   │   ├── dabai.launch  # (need to be modified) Launch file to launch the camera node & combine camera and arm TF trees 
│   │   ├── [other packages]/ # Additional vision-related packages (not used in this project)
├── launch_all.sh            # Script to launch all nodes (camera, arm, TF, YOLO, picking)
└── README.md                # Project documentation
```

---

## 🛠 System Requirements

- **Hardware**:
  - RK3588-based device (e.g., OrangePi)
  - Orbbec Astra Dabai camera (for RGB and depth data)
  - Piper robotic arm (with gripper)
- **Software**:
  - Ubuntu 20.04
  - ROS Noetic (or compatible version)
  - YOLOv5 dependencies ([airockchip/yolov5](https://github.com/airockchip/yolov5))
  - RKNN Toolkit for model conversion ([rknn-toolkit2](https://github.com/rockchip-linux/rknn-toolkit2))
  - Orbbec Astra ROS driver ([orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera))
  - Piper ROS SDK ([agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros))

---

## 🚀 Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/mu9enn/eyes_piper.git
cd eyes_piper
```

### 2. Install Dependencies
- **ROS Noetic**:

- **Orbbec Astra Camera Driver**:
  Follow the setup instructions in [orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera) to configure the `piper_ros` workspace.
- **Piper ROS SDK**:
  Follow the setup instructions in [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros) to configure the `piper_ros` workspace.
- **YOLOv5 and RKNN Toolkit**:
  - Install dependencies from [airockchip/yolov5](https://github.com/airockchip/yolov5).
  - Follow [README_rkopt.md](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md) to convert `.pt` to `.onnx`.
  - Use [rknn_model_zoo yolov5 example](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov5) to convert `.onnx` to `.rknn` for RK3588 NPU inference.

### 3. Configure Workspaces
Use `catkin_make` to setup:
- **piper_ros**: [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros)
- **ros_astra_camera**: [orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera):


### 4. Model Training and Deployment
- **Train YOLOv5 Model (on personal computer)**:
  - Use [airockchip/yolov5](https://github.com/airockchip/yolov5) to train a `.pt` model on your dataset.
- **Convert Model (on personal computer)**:
  - Convert `.pt` to `.onnx` following [README_rkopt.md](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md).
  - Convert `.onnx` to `.rknn` using [rknn_model_zoo yolov5 example](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov5).
- **Deploy on RK3588**:
  - Place the `.rknn` model in the appropriate directory (e.g., `vision_ros/src/yolov5/model/`).
  - Update the model path in `ros_detect_sunx.py` if necessary.

---

## 📚 File Descriptions

- **`piper_ros`**: ROS workspace for Piper robotic arm control.
  - **piper_gopick**: Custom ROS node (`piper_ros/src/piper_gopick`) that processes fruit positions from the camera and executes picking actions using the Piper arm.
  - **Configuration**: Identical to [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros). Follow its setup for CAN bus and MoveIt! configuration.

- **`vision_ros`**: ROS workspace for camera operation and YOLO detection.
  - **astra_camera**: Primary package used for Orbbec Astra Dabai camera, providing RGB and depth image streams.
  - **Other Packages**: Additional vision-related packages exist in `vision_ros/src` but are not utilized in this project.

- **`arm_tf.launch`**: ROS launch file that integrates the TF (transform) trees of the Piper robotic arm and Orbbec camera, ensuring accurate 3D coordinate transformations between camera and arm base frames.

- **`launch_all.sh`**: Bash script to launch all necessary nodes in one command:
  - Orbbec camera node (`astra_camera`)
  - Piper arm control node
  - TF tree integration (`arm_tf`)
  - YOLO detection node (`ros_detect_sunx.py`)
  - Fruit picking node (`pickone_sunx.py`)

---

## 🛠 Implementation

The system operates by coordinating the Orbbec camera’s YOLO-based detection with the Piper robotic arm’s picking actions. Below is the step-by-step workflow to run the fruit-picking system.

### First, combine camera and arm TF trees
Add a this to `ros_astra_camera/launch/dabai.launch`:
```
todo

``` 

### Terminal 1: Launch the System
1. **Activate the CAN Bus for Piper Arm**:
   ```bash
   cd piper_ros
   source devel/setup.bash
   bash can_activate.sh can0 1000000
   ```


2. **Launch All Nodes**:
   ```bash
   cd ..
   bash launch_all.sh
   ```
   - **Purpose**: Starts the camera node (`astra_camera`), Piper arm node, TF integration (`arm_tf`), YOLO detection (`ros_detect_sunx.py`), and picking node (`pickone_sunx.py`).
   - **Behavior**: The system waits for a `1` signal on the `/wheels/arrive` topic to trigger fruit picking for the current tree.

### Terminal 2: Trigger Picking
1. **Manually Publish Trigger Signal**:
   ```bash
   rostopic pub --once /wheels/arrive std_msgs/Int32 1
   ```
   - **Purpose**: Sends a `1` to the `/wheels/arrive` topic to initiate the fruit-picking sequence for the current tree.
   - **Note**: This simulates a signal from a mobile platform or navigation system indicating the arm is in position.

### Workflow Details
- **Detection**:
  - The `ros_detect_sunx.py` node processes RGB and depth images from the Orbbec camera, running YOLOv5 on the RK3588 NPU to detect fruits (`m` class) and compute their 3D coordinates in the `base_link` frame.
  - Coordinates are published to `/yolo/m_xyz` after de-duplication.
- **Picking**:
  - The `pickone_sunx.py` node (in `piper_gopick`) subscribes to `/yolo/m_xyz`, transforms coordinates to the arm’s workspace, and commands the Piper arm to pick and place fruits.
  - The arm uses MoveIt! for motion planning, with TF integration ensuring accurate positioning.
- **Trigger**:
  - The `/wheels/arrive` signal initiates a cycle of observation and picking for one tree, with multiple observation poses to detect all reachable fruits.

---

## 📈 Usage Notes
- **Environment Setup**: Ensure the RK3588 board is properly configured with the RKNN Toolkit and ROS Noetic.
- **Model Path**: Verify the `.rknn` model path in `ros_detect_sunx.py` matches your deployment setup.
- **TF Calibration**: Check `arm_tf.launch` for correct transform parameters between the camera and arm base to avoid localization errors.
- **Testing**: Use `rostopic pub` to test the system incrementally, ensuring each node (camera, YOLO, arm) functions correctly before full operation.

---

## 🙏 Acknowledgments

- **Orbbec**: For the [Astra camera ROS driver](https://github.com/orbbec/ros_astra_camera).
- **AgileX Robotics**: For the [Piper ROS SDK](https://github.com/agilexrobotics/piper_ros).
- **AI Rockchip**: For the [YOLOv5 implementation](https://github.com/airockchip/yolov5) and [RKNN model zoo](https://github.com/airockchip/rknn_model_zoo).
- **Rockchip**: For RK3588 hardware and [RKNN Toolkit](https://github.com/rockchip-linux/rknn-toolkit2).
