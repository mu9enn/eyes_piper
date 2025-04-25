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
1. 克隆代码，也可以参考[ros_astra_camera原链接](https://github.com/orbbec/ros_astra_camera)：
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
2. 编辑 `ros_detect.py`，（即本仓库的`ros_detect.py`）。
   
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
常规的对YOLO进行数据集标注通常使用[LabelImg](https://github.com/HumanSignal/labelImg)或其他工具手动标注。

这里提供一个基于[Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything)的数据集生成+标注方法，数据集质量可能比较低，但可以快速将YOLO模型微调到目标任务，方便同步整车的联调工作。

- 创建虚拟环境并安装Lang-SAM
```bash
conda activate 
conda create -n langsam python=3.11
conda activate langsam
pip install torch==2.4.1 torchvision==0.19.1 --extra-index-url https://download.pytorch.org/whl/cu124
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git
```
- 安装YOLO训练要求的依赖项
```bash
cd ~/yoloworkflow/yolov5
pip install -r requirements.txt
```
### - 逐步调用这里写好的python文件完成数据集的生成:
1. 拍一些目标的图片放在`captured`文件夹，收集一些其他的图片作为背景放在`backgrounds`文件夹下（建议有4000张以上且样式多样化一些）。
2. 确保环境和目录是对的
```bash
conda activate langsam
cd ~/yolo_workflow/dataset_workflow
```
3. 对`captured`文件夹里的图片进行分割和标注，示例代码完成的是针对红色和黄色的`pepper`进行分割，然后根据颜色把红色标注为`1`成熟，黄色标注为`0`不成熟，对不同任务需要对此代码进行具体修改。
```bash
python segment_label.py
```
4. 把分割后的图像和背景都resize成640x640大小
```bash
python resize_pics.py
```
5. 将分割后的图像和背景进行粘贴，同时生成bounding box标注
```bash
python synthesize_images.py
```
6. 标注后可以查看某些文件的bounding box是否准确
```bash
python test_data.py
```
7. 把数据分成训练和验证集
```bash
python train_val.py
```

### - 用生成的数据进行YOLO的训练

1. 为数据集编写`.yaml`文件
```bash
cd ~/yolo_workflow/yolov5/data
touch mydataset.yaml
```
2. 编辑`.yaml`文件，下面展示仍然只是示例，需要根据具体情况调整
```yaml
# Train/val/test sets as 1) dir: path/to/imgs, 2) file: path/to/imgs.txt, or 3) list: [path/to/imgs1, path/to/imgs2, ..]
path: /home/sunx/code_proj/dataset_workflow/yolo_data  # dataset root dir
train: images/train  # train images (relative to 'path')
val: images/val  # val images (relative to 'path')
test:  # test images (optional)

# Classes
nc: 2  # number of classes
names: ['immature', 'mature']  # class names
```
3. 运行训练
```bash
cd ~/yolo_workflow/yolov5
python train.py --data mydataset.yaml --epochs 300 --weights '' --cfg yolov5s.yaml --batch-size 128
```
训练结果保存在`yolov5/runs/train/exp$num$/weights`目录下,把`best.pt`复制到`yolov5`文件夹下，在`ros_detect.py`中修改调用的`.pt`权重文件即可用训练好的权重进行目标检测。
