# Piper Fruit Picking Robot 

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) ![ros](https://img.shields.io/badge/ROS-noetic-blue.svg) ![status](https://img.shields.io/badge/Pass-blue.svg)

> 本项目集成 **YOLO 目标检测**、**机械臂 MoveIt 控制** 与 **3D视觉定位**，实现对成熟果实的自动识别与抓取。

---

⭐要讲yolo检测移植到rk3588板上需参考[https://github.com/airockchip/yolov5/blob/master/README_rkopt.md](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md)

## 🧭 Quick Start

### 🔧 编译工作空间

```bash
cd eyes_piper/piper_ros
catkin_make
```

### 安装 udev 规则

```
cd eyes_piper/piper_ros
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo udevadm trigger
```

### 安装 Python 依赖

```
pip install -r requirement.txt
```


---

### 🖥️ 启动流程（5个终端）

#### **终端1：启动相机与YOLO检测算法**

```bash
source devel/setup.bash
roslaunch astra_camera dabai.launch
```

> 发布检测目标坐标至话题 `/yolo/label_xyz`

---

#### **终端2：CAN初始化并使能机械臂**

```bash
source devel/setup.bash
bash can_activate.sh can0 1000000
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

---

#### **终端3：启动 MoveIt 控制服务（可选启动 RViz）**

```bash
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

---

#### **终端4：发布 camera\_link 与 gripper\_base 之间的静态TF & 加载YOLO**

```bash
source devel/setup.bash
roslaunch astra_camera cam_function.launch
```

设置参数开启目标检测与图像显示：

```bash
source devel/setup.bash
rosparam set /go_detect true
rosparam set /yolo/show_image true
```


---

#### **终端5：执行定位与抓取任务（单棵树）**

```bash
source devel/setup.bash
rosrun moveit_ctrl pick_one.py
```

---

## 🎯 功能概述

* ✅ **果实检测与3D定位**：YOLO模型检测成熟果实，结合深度图还原空间坐标。
* ✅ **目标姿态跟随**：抓取前自动调整 base 关节，使 gripper 始终正对目标。
* ✅ **MoveIt精确控制**：通过 `JointMoveitCtrl` 服务完成带姿态末端抓取。
* ✅ **夹爪控制**：抓取前打开，接近目标后可进一步闭合。

---

## 📦 项目结构简述 (piper_ros/src)

```bash
├── ros_astra_camera/       # 相机ros包
│   ├── scripts/ 
│   │   ├── ros_detect.py/      # 在相机上执行yolo检测的rosservice脚本文件⭐
├── piper_moveit/ 
│   ├── moveit_ctrl/     # MoveIt控制服务与任务执行脚本
│       ├── scripts/        # pick_one.py 所在目录⭐
│   ├── piper_with_gripper_moveit/
│   │   ├── launch/
│   │   │   ├── static_tf.launch     # camera_link 与 gripper_base 的坐标变换⭐
│   │   │   └── demo.launch          # 机械臂MoveIt 控制器启动文件
├── can_activate.sh         # 初始化CAN设备脚本
└── README.md
```

---

## 🔗 参考资料

* **奥比中光相机驱动**
  [https://github.com/orbbec/ros\_astra\_camera](https://github.com/orbbec/ros_astra_camera)

* **松灵 Piper 机械臂 API**
  [https://github.com/agilexrobotics/piper\_ros](https://github.com/agilexrobotics/)

* **手眼标定 easy\_handeye**
  [https://github.com/IFL-CAMP/easy\_handeye](https://github.com/IFL-CAMP/easy_handeye)


