# 📸 Orbbec Dabai Camera with YOLOv5 Real-Time Object Detection Guide

**Environment**: Ubuntu 20.04, ROS Noetic, Orbbec Dabai Camera  
**Objective**: Configure the Orbbec Dabai camera to acquire RGB, depth images, and point clouds, and integrate YOLOv5 for real-time object detection.  
**Date**: April 25, 2025

---

## 📖 Overview

This guide provides step-by-step instructions to set up the **Orbbec Dabai camera** with **ROS Noetic** on Ubuntu 20.04, capturing RGB and depth images for real-time object detection using **YOLOv5**. The system is designed for applications such as precision agriculture (e.g., fruit detection) and robotic perception, leveraging the Orbbec camera's capabilities and YOLOv5's efficient object detection framework. Additionally, it includes a dataset generation pipeline using **Lang-SAM** for rapid prototyping and model fine-tuning.

---

## 🛠 Prerequisites

- **Hardware**: Orbbec Dabai camera, computer with NVIDIA GPU (optional for YOLOv5 training).
- **Software**:
  - Ubuntu 20.04
  - ROS Noetic
  - CUDA (for GPU-accelerated YOLOv5 training)
  - Anaconda (for managing Python environments)
  - Python 3.11 (for Lang-SAM dataset generation)
- **Repositories**:
  - [orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera)
  - [ultralytics/yolov5](https://github.com/ultralytics/yolov5)
  - [luca-medeiros/lang-segment-anything](https://github.com/luca-medeiros/lang-segment-anything)

---

## 0. Install CUDA and Anaconda

### 0.1 Install CUDA
- Follow NVIDIA's official guide to install CUDA:
  - Navigate to **System Settings > Software & Updates > Additional Drivers**.
  - Select the appropriate **NVIDIA driver** and apply changes.
  - Verify installation:
    ```bash
    nvidia-smi
    ```
  - Ensure a table of GPU information is displayed.

### 0.2 Install Anaconda
- Download the installer from the [Anaconda website](https://www.anaconda.com/download).
- Run the installer:
  ```bash
  bash Anaconda-latest-Linux-x86_64.sh
  ```
- **Important**: When prompted to initialize Anaconda in the terminal, select **no** to avoid modifying the default shell configuration.

---

## 1. Install ROS Noetic and Camera Dependencies

### 1.1 Install ROS Noetic
- Follow the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).
- Configure the ROS environment:
  ```bash
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

### 1.2 Install Camera Dependencies
1. Install required ROS and system packages:
   ```bash
   sudo apt update
   sudo apt install libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
   ros-noetic-image-transport ros-noetic-image-publisher libusb-1.0-0-dev libeigen3-dev \
   ros-noetic-backward-ros libdw-dev
   ```
2. Install `libuvc`:
   ```bash
   git clone https://github.com/libuvc/libuvc.git
   cd libuvc
   mkdir build && cd build
   cmake .. && make -j4
   sudo make install
   sudo ldconfig
   ```

---

## 2. Configure Orbbec Dabai Camera

### 2.1 Create ROS Workspace
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

### 2.2 Install `ros_astra_camera`
1. Clone the repository:
   ```bash
   git clone https://github.com/orbbec/ros_astra_camera.git
   ```
   - Reference: [orbbec/ros_astra_camera](https://github.com/orbbec/ros_astra_camera)
2. Build the workspace:
   ```bash
   cd ~/ros_ws
   catkin_make
   ```

### 2.3 Set Up Udev Rules
```bash
cd ~/ros_ws
source devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo udevadm trigger
```

---

## 3. Start Camera and Verify

### 3.1 Launch Camera Node
- In **Terminal 1**:
  ```bash
  cd ~/ros_ws
  source devel/setup.bash
  roslaunch astra_camera dabai.launch
  ```

### 3.2 Visualize Images in RViz
- In **Terminal 2**:
  ```bash
  cd ~/ros_ws
  source devel/setup.bash
  rviz
  ```
- In RViz:
  - Click **Add** > **By topic**.
  - Add **Image** displays for:
    - `/camera/color/image_raw` (RGB image)
    - `/camera/depth/image_raw` (Depth image)

---

## 4. Configure YOLOv5 for Real-Time Object Detection

### 4.1 Install YOLOv5
1. Clone the project and YOLOv5 repositories:
   ```bash
   sudo apt update
   sudo apt install git
   cd ~/
   git clone https://github.com/mu9enn/yolo_workflow.git
   cd ~/yolo_workflow
   git clone https://github.com/ultralytics/yolov5.git
   cd ~/yolo_workflow/yolov5
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### 4.2 Create ROS-YOLOv5 Detection Script
1. Create the script:
   ```bash
   cd ~/yolo_workflow/yolov5
   touch ros_detect.py
   ```
2. Copy the contents of `ros_detect.py` from this repository (e.g., [yolo_workflow/ros_detect.py](yolo_workflow/ros_detect.py)).
3. Add execution permissions:
   ```bash
   chmod +x ros_detect.py
   ```

### 4.3 Install ROS Dependencies
```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-rospy
```

---

## 5. Run Real-Time Object Detection

### 5.1 Start ROS Master
- In **Terminal 1**:
  ```bash
  roscore
  ```

### 5.2 Launch Camera Node
- In **Terminal 2**:
  ```bash
  cd ~/ros_ws
  source devel/setup.bash
  roslaunch astra_camera dabai.launch
  ```

### 5.3 Run YOLOv5 Detection
- In **Terminal 3**:
  ```bash
  cd ~/yolo_workflow/yolov5
  source ~/ros_ws/devel/setup.bash
  python3 ros_detect.py
  ```

### 5.4 View Detection Results
- **Real-Time Window**: Detection results (bounding boxes and labels) are displayed in an OpenCV window.
- **ROS Topic**: Detected images are published to `/yolo/detections`. View them using RViz or `rqt_image_view`:
  ```bash
  rqt_image_view /yolo/detections
  ```

---

## 6. Dataset Generation and YOLOv5 Training

### 6.1 Dataset Generation with Lang-SAM
For rapid prototyping, a dataset generation pipeline using [Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything) is provided. This method generates synthetic datasets with potentially lower quality but enables quick fine-tuning of YOLOv5 models for specific tasks, facilitating integration with robotic systems.

1. **Create Virtual Environment**:
   ```bash
   conda create -n langsam python=3.11
   conda activate langsam
   pip install torch==2.4.1 torchvision==0.19.1 --extra-index-url https://download.pytorch.org/whl/cu124
   pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git
   ```

2. **Install YOLOv5 Dependencies**:
   ```bash
   cd ~/yolo_workflow/yolov5
   pip install -r requirements.txt
   ```

3. **Generate Dataset**:
   - Place target object images in `dataset_workflow/captured/` (e.g., images of fruits).
   - Collect diverse background images (4000+ recommended) in `dataset_workflow/backgrounds/`.
   - Ensure the environment and directory are set:
     ```bash
     conda activate langsam
     cd ~/yolo_workflow/dataset_workflow
     ```
   - Run the following scripts in sequence:
     1. **Segment and Label Objects**:
        ```bash
        python segment_label.py
        ```
        - Segments objects (e.g., red/yellow peppers) using Lang-SAM.
        - Labels red peppers as `1` (mature) and yellow as `0` (immature). Modify the script for your specific task.
     2. **Resize Images**:
        ```bash
        python resize_pics.py
        ```
        - Resizes segmented objects and backgrounds to 640x640 for YOLOv5 compatibility.
     3. **Synthesize Images**:
        ```bash
        python synthesize_images.py
        ```
        - Pastes segmented objects onto backgrounds and generates bounding box annotations.
     4. **Verify Annotations**:
        ```bash
        python test_data.py
        ```
        - Checks bounding box accuracy for generated images.
     5. **Split Dataset**:
        ```bash
        python train_val.py
        ```
        - Splits data into training and validation sets.

### 6.2 Train YOLOv5 Model
1. **Create Dataset Configuration**:
   ```bash
   cd ~/yolo_workflow/yolov5/data
   touch mydataset.yaml
   ```
2. **Edit `mydataset.yaml`** (example, adjust for your task):
   ```yaml
   path: /home/sunx/code_proj/dataset_workflow/yolo_data  # Dataset root directory
   train: images/train  # Training images (relative to 'path')
   val: images/val      # Validation images (relative to 'path')
   test:                # Test images (optional)

   # Classes
   nc: 2                # Number of classes
   names: ['immature', 'mature']  # Class names
   ```
3. **Run Training**:
   ```bash
   cd ~/yolo_workflow/yolov5
   python train.py --data mydataset.yaml --epochs 300 --weights '' --cfg yolov5s.yaml --batch-size 128
   ```
   - Training results are saved in `yolov5/runs/train/exp$num$/weights`.
   - Copy `best.pt` to `yolov5/` and update the weight path in `ros_detect.py` for detection.

---

## 🤝 Contributing

Contributions are welcome! To contribute:
1. Fork the repository and create a feature branch.
2. Make changes, ensuring compatibility with ROS Noetic and Orbbec Dabai.
3. Submit a pull request with a detailed description.
4. Update documentation for new features.

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE). See the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Orbbec**: For the [Astra camera ROS driver](https://github.com/orbbec/ros_astra_camera).
- **Ultralytics**: For the [YOLOv5 framework](https://github.com/ultralytics/yolov5).
- **Lang-SAM**: For the [segmentation toolkit](https://github.com/luca-medeiros/lang-segment-anything).
- **LabelImg**: For manual annotation support ([HumanSignal/labelImg](https://github.com/HumanSignal/labelImg)).