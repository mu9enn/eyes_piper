# 🦾 Eyes_Piper: YOLO-Based Object Detection and Piper Robotic Arm Collaboration System

[![License](https://img.shields.io/github/license/mu9enn/eyes_piper)](https://github.com/mu9enn/eyes_piper/blob/main/LICENSE)
[![Stars](https://img.shields.io/github/stars/mu9enn/eyes_piper?style=social)](https://github.com/mu9enn/eyes_piper/stargazers)
[![Last Commit](https://img.shields.io/github/last-commit/mu9enn/eyes_piper)](https://github.com/mu9enn/eyes_piper/commits/main)

The **Eyes_Piper** project integrates **YOLOv5** for object detection with the **Orbbec camera** and **Piper robotic arm** to achieve autonomous perception and manipulation in a collaborative system. It supports deployment on two platforms: **RK3588-based embedded systems** (e.g., OrangePi) and **Ubuntu PCs**, catering to diverse development and deployment environments. This project is designed for applications in precision agriculture, industrial automation, and robotic research, leveraging real-time computer vision and robotic control.[](https://www.nature.com/articles/s41598-025-88439-w)

---

## 📂 Project Structure

```bash
├── catkin_ros/               # Deployment and runtime environment for RK3588-based systems (e.g., OrangePi)
├── ubuntu_pc/                # Development and debugging environment for Ubuntu PCs
│   ├── dataset_workflow/     # Tools for automated dataset creation and annotation for YOLO training
│   │   ├── sam_syth/        # Synthetic dataset generation using Lang-SAM for object segmentation and background pasting
│   │   ├── dino_label/      # Automated object detection with Grounding-DINO, followed by manual refinement scripting
```

---

## 🤖 Project Modules

### 1. **Orbbec Astra Dabai Camera**
   - **Functionality**: Captures RGB and depth images for real-time object detection and 3D localization.
   - **Interface Source**: [Orbbec ROS Astra Camera](https://github.com/orbbec/ros_astra_camera)
   - **Description**: Utilizes the Orbbec Astra camera to acquire high-quality RGB and depth data, enabling precise object detection and spatial mapping for robotic tasks.

### 2. **Piper Robotic Arm ROS API**
   - **Functionality**: Controls the Piper robotic arm for motion planning and manipulation tasks.
   - **Control Interface Source**: [AgileX Robotics Piper ROS](https://github.com/agilexrobotics/piper_ros)[](https://github.com/agilexrobotics/piper_sdk)
   - **Description**: Integrates with MoveIt! for advanced motion planning and supports multiple end-effectors, enabling tasks such as object localization, grasping, and placement in coordination with the detection module.

---

## 📖 Detailed Usage Instructions

This project is organized into subfolders, each with its own `README.md` providing detailed setup and usage instructions. Below is a brief overview of each component and links to their respective documentation.

1. **[catkin_ros/](catkin_ros/README.md)**  
   - **Description**: Contains the ROS-based deployment environment tailored for RK3588 platforms (e.g., OrangePi). This folder includes scripts, configurations, and dependencies for running the YOLOv5 detection pipeline and Piper robotic arm control on embedded systems. Optimized for real-time performance using the RK3588’s NPU.[](https://github.com/Applied-Deep-Learning-Lab/Yolov5_RK3588)
   - **Use Case**: Ideal for deploying the system on resource-constrained devices in production environments, such as autonomous fruit picking or industrial automation.

2. **[ubuntu_pc/](ubuntu_pc/README.md)**  
   - **Description**: Provides a development and debugging environment for Ubuntu PCs, enabling researchers and developers to train models, test algorithms, and simulate robotic operations. This folder includes tools for dataset preparation and model training.
   - **Use Case**: Suited for prototyping, model development, and testing before deployment on RK3588 systems.

3. **[ubuntu_pc/dataset_workflow/](ubuntu_pc/dataset_workflow/README.md)**  
   - **Description**: Offers tools for creating and annotating datasets for YOLOv5 training, streamlining the process of preparing high-quality training data.
   - **Submodules**:
     - **[sam_syth/](ubuntu_pc/dataset_workflow/sam_syth/README.md)**: Generates synthetic datasets by using Lang-SAM to segment target objects and paste them onto diverse background images, enhancing dataset variety and robustness.
     - **[dino_label/](ubuntu_pc/dataset_workflow/dino_label/README.md)**: Employs Grounding-DINO for automated object detection, followed by scripting for manual refinement to ensure accurate annotations for training YOLOv5 models.
   - **Use Case**: Facilitates efficient dataset creation for training custom YOLOv5 models, critical for tasks like detecting specific objects (e.g., fruits) in varied environments.

---

## 🚀 Getting Started

To get started with the **Eyes_Piper** project, follow these steps:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/mu9enn/eyes_piper.git
   cd eyes_piper
   ```

2. **Select Your Platform**:
   > If you are participating in the **RAICOM Robot Developer Competition under the “Fruit Recognition” track** and have limited experience with Ubuntu/ROS, we recommend first testing the programs by connecting the robotic arm and camera to your personal computer, and gradually building up to the full functionality.
   - For **Ubuntu PC development**, navigate to `ubuntu_pc/` and refer to its [README.md](ubuntu_pc/README.md).
   - For **RK3588 deployment** (e.g., OrangePi), navigate to `catkin_ros/` and follow the instructions in its [README.md](catkin_ros/README.md).


3. **Install Dependencies**:
   - Ensure ROS noetic is installed for both platforms.
   - Install required libraries for YOLOv5 (e.g., PyTorch, OpenCV) and the RK3588 NPU toolkit (e.g., rknn-toolkit2).
   - Set up the Orbbec Astra camera and Piper robotic arm SDKs as per their respective documentation.

4. **Run the System**:
   - Launch the ROS nodes for camera data acquisition, YOLOv5 detection, and Piper arm control.
   - Use the provided scripts in each subfolder to execute detection and manipulation tasks.

---

## 📚 Additional Resources

- **Orbbec Astra Camera Documentation**: [Orbbec Developer Portal](https://developer.orbbec.com.cn/)
- **Piper Robotic Arm SDK**: [AgileX Robotics Piper SDK](https://github.com/agilexrobotics/piper_sdk)
- **YOLOv5 Documentation**: [Ultralytics YOLOv5](https://docs.ultralytics.com/)
- **RK3588 NPU Support**: Refer to [rknn-toolkit2](https://github.com/rockchip-linux/rknn-toolkit2) for model conversion and optimization.
