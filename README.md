# Orbbec Dabai 相机与 YOLOv5 实时目标检测指南

**适用环境**：Ubuntu 20.04，ROS Noetic，Orbbec Dabai 相机  
**目标**：配置 Orbbec Dabai 相机，获取 color/depth 图像和点云，通过 YOLOv5 实时检测目标。  
**日期**：2025年4月11日

## 0.安装CUDA，Anaconda
- CUDA可以直接参照网络教程，以安装好后在终端输入`nvidia-smi`后有显示为准
- Anaconda直接在官网下载安装，**注意安装过程中询问`是否在终端中默认调用conda`时选择`no`”**


## 1. 安装 ROS Noetic 和相机依赖

### 1.1 安装 ROS Noetic
- 参考 [ROS 官方安装指南](http://wiki.ros.org/noetic/Installation/Ubuntu) 安装 ROS Noetic。
- 确保已配置 ROS 环境：
  ```bash
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

### 1.2 安装相机依赖
1. 安装基本依赖：
   ```bash
   sudo apt install libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
   ros-noetic-image-transport ros-noetic-image-publisher libusb-1.0-0-dev libeigen3-dev \
   ros-noetic-backward-ros libdw-dev
   ```
2. 安装 `libuvc`：
   ```bash
   git clone https://github.com/libuvc/libuvc.git
   cd libuvc
   mkdir build && cd build
   cmake .. && make -j4
   sudo make install
   sudo ldconfig
   ```

## 2. 配置 Orbbec Dabai 相机

### 2.1 创建 ROS 工作空间
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

### 2.2 安装 `ros_astra_camera`
1. 克隆代码：
   ```bash
   git clone https://github.com/orbbec/ros_astra_camera.git
   ```
2. 构建工作空间：
   ```bash
   cd ~/ros_ws
   catkin_make
   ```

### 2.3 设置 Udev 规则
```bash
cd ~/ros_ws
source devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo udevadm trigger
```

## 3. 启动相机并验证

### 3.1 启动相机节点
1. 打开终端 1：
   ```bash
   cd ~/ros_ws
   source devel/setup.bash
   roslaunch astra_camera dabai.launch
   ```

### 3.2 使用 RViz 查看图像
1. 打开终端 2：
   ```bash
   cd ~/ros_ws
   source devel/setup.bash
   rviz
   ```
2. 在 RViz 中添加`Add`显示：
   - **`Image`**：选择话题 `/camera/color/image_raw`（RGB 图像）。
   - **`Image`**：选择话题 `/camera/depth/image_raw`（深度图像）。

## 4. 配置 YOLOv5 实时目标检测

### 4.1 安装 YOLOv5
1. 克隆代码：
   ```bash
   git clone https://github.com/ultralytics/yolov5.git
   cd ~/yolo_workflow/yolov5
   ```
2. 安装依赖：
   ```bash
   pip install -r requirements.txt
   ```

### 4.2 创建 ROS-YOLOv5 检测脚本
1. 创建脚本：
   ```bash
   cd ~/yolo_workflow/yolov5
   touch ros_detect.py
   ```
2. 编辑 `ros_detect.py`，（将本仓库的）。
   
3. 添加执行权限：
   ```bash
   chmod +x ros_detect.py
   ```

### 4.3 安装 ROS 依赖
```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-rospy
```

## 5. 运行实时目标检测

### 5.1 启动 ROS Master
- 打开终端 1：
  ```bash
  roscore
  ```

### 5.2 启动相机节点
- 打开终端 2：
  ```bash
  cd ~/ros_ws
  source devel/setup.bash
  roslaunch astra_camera dabai.launch
  ```

### 5.3 运行 YOLOv5 检测
- 打开终端 3：
  ```bash
  cd ~/yolo_workflow/yolov5
  source ~/ros_ws/devel/setup.bash
  python3 ros_detect.py
  ```

### 5.4 查看检测结果
- **实时窗口**：检测结果（边界框和标签）会显示在 OpenCV 窗口中。
- **ROS 话题**：检测图像发布到 `/yolo/detections`，可用 RViz 或 `rqt_image_view` 查看：
  ```bash
  rqt_image_view /yolo/detections
  ```

## 6. 数据采集

这里提供一个基于[Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything)的数据集生成+标注方法，数据集质量可能比较低，但可以快速将YOLO模型微调到目标任务，方便同步整车的联调工作。

- 创建虚拟环境并安装Lang-SAM
```bash
conda activate 
conda create -n langsam python=3.11
pip install torch==2.4.1 torchvision==0.19.1 --extra-index-url https://download.pytorch.org/whl/cu124
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git
```
- 安装YOLO训练要求的依赖项
```bash
cd ~/yoloworkflow/yolov5
pip install -r requirements.txt
```
