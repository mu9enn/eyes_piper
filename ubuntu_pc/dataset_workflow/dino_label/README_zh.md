# 📊 使用 Grounding DINO 为 YOLOv5 生成数据集标注

![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) ![Python](https://img.shields.io/badge/Python-3.8+-blue.svg) ![YOLOv5](https://img.shields.io/badge/YOLOv5-v7.0-brightgreen.svg)

本目录提供了一个使用 **Grounding DINO**（基于文本提示的目标检测模型）为 YOLOv5 训练数据集生成和优化标注的流水线。该流水线通过自动化生成初始边界框标注并支持手动优化以提高精度，适用于果实检测等机器人应用场景。生成的标注采用 YOLO 兼容格式，可无缝集成到 YOLOv5 训练流程中。

---

## 📖 概述

`dino_label` 目录通过以下步骤实现 YOLOv5 数据集的自动化和优化标注：
1. 预处理图像以统一格式和大小。
2. 使用 **Grounding DINO** 根据文本提示生成初始边界框标注。
3. 支持手动优化标注以提高精度。
4. 将数据集拆分为训练集和验证集。

该流水线旨在快速生成数据集，平衡自动化和手动控制，特别适合于需要快速微调 YOLOv5 模型的特定目标检测任务（如检测成熟果实）。

---

## 📂 目录结构

```bash
├── images/            # 存放待标注的输入图像
├── labels/            # 存放生成的 YOLO 格式标注文件
├── better_anno.py     # 使用 OpenCV 进行手动优化标注的脚本
├── data.yaml          # YOLOv5 数据集配置文件
├── labelling.py       # 使用 Grounding DINO 进行自动化标注的脚本
├── png.py             # 将图像转换为 PNG 格式的脚本
├── resize.py          # 将图像调整为统一大小（如 640x640）的脚本
├── train_val.py       # 将数据集拆分为训练集和验证集的脚本
├── README.md          # 本文档
```

- **`images/`**：存放待标注的原始图像（如果实的照片）。
- **`labels/`**：存放由 `labelling.py` 生成并由 `better_anno.py` 优化后的 YOLO 格式 `.txt` 标注文件。

---

## 🛠 前置条件

- **软件**：
  - Ubuntu 20.04
  - Python 3.8 或更高版本
  - [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO) 依赖
  - [YOLOv5](https://github.com/ultralytics/yolov5) 依赖
  - OpenCV (`cv2`) 和 NumPy 用于手动标注
- **硬件**：具有足够存储空间的 PC，用于存储图像和标注文件。
- **依赖**：
  - 安装 Grounding DINO 和 YOLOv5 的依赖（见[设置步骤](#设置步骤)）。
- **输入数据**：
  - 将待标注的图像放置在 `images/` 目录下。

---

## 🚀 标注流水线

### 设置步骤
1. **克隆仓库**：
   ```bash
   git clone https://github.com/mu9enn/eyes_piper.git
   cd eyes_piper/ubuntu_pc/dataset_workflow/dino_label
   ```

2. **安装依赖**：
   - 安装 YOLOv5 依赖：
     ```bash
     cd ~/yolo_workflow/yolov5
     pip install -r requirements.txt
     ```
   - 安装 Grounding DINO：
     ```bash
     pip install -U git+https://github.com/IDEA-Research/GroundingDINO.git
     ```
   - 安装 OpenCV 和 NumPy：
     ```bash
     pip install opencv-python numpy
     ```

3. **准备输入图像**：
   - 将待标注的图像放置在 `images/` 目录下。
   - 确保图像格式兼容（如 JPG、PNG）。

### 运行标注脚本
进入 `dino_label` 目录：
```bash
cd ~/yolo_workflow/dataset_workflow/dino_label
```

按以下顺序运行脚本：

1. **将图像转换为 PNG 格式**：
   ```bash
   python png.py
   ```
   - **功能**：将 `images/` 目录中的所有图像转换为 PNG 格式以确保一致性。
   - **输出**：在 `images/` 中覆盖或创建 PNG 格式图像。

2. **调整图像大小**：
   ```bash
   python resize.py
   ```
   - **功能**：将 `images/` 目录中的图像调整为统一大小（如 640x640），以兼容 YOLOv5。
   - **输出**：更新 `images/` 中的图像为调整后的版本。

3. **使用 Grounding DINO 自动标注**：
   ```bash
   python labelling.py
   ```
   - **功能**：使用 Grounding DINO 根据文本提示（如“红辣椒”、“黄辣椒”）生成边界框标注。
   - **自定义**：修改 `labelling.py` 中的 `TEXT_PROMPT` 变量以匹配目标物体（如 `TEXT_PROMPT = "成熟果实"`）。
   - **输出**：在 `labels/` 目录中生成与每个图像对应的 YOLO 格式 `.txt` 文件。

4. **手动优化标注**：
   ```bash
   python better_anno.py
   ```
   - **功能**：使用 OpenCV 和 NumPy 显示图像，支持手动调整边界框以提高精度。
   - **使用方法**：按照屏幕提示交互式优化标注。
   - **输出**：更新 `labels/` 目录中的 `.txt` 文件，包含优化后的边界框。

5. **拆分数据集**：
   ```bash
   python train_val.py
   ```
   - **功能**：将图像和标注拆分为训练集和验证集，通常存储在 `images/train/`、`images/val/`、`labels/train/` 和 `labels/val/` 中。
   - **输出**：为 YOLOv5 训练准备好组织化的数据集结构。

---

## 🧠 使用生成的数据训练 YOLOv5

### 1. 编辑数据集配置文件
`data.yaml` 文件定义了 YOLOv5 训练的数据集结构。示例内容（需根据实际情况调整路径和类别名称）：
```yaml
path: /home/sunx/code_proj/dataset_workflow/dino_label  # 数据集根目录
train: images/train  # 训练图像（相对于 'path'）
val: images/val      # 验证图像（相对于 'path'）
test:                # 测试图像（可选）

# 类别
nc: 2                # 类别数量
names: ['immature', 'mature']  # 类别名称
```

如需创建或修改 `data.yaml`：
```bash
cd ~/yolo_workflow/dataset_workflow/dino_label
touch data.yaml
```

### 2. 运行训练
```bash
cd ~/yolo_workflow/yolov5
python train.py --data ../dataset_workflow/dino_label/data.yaml --epochs 300 --weights '' --cfg yolov5s.yaml --batch-size 128
```
- **输出**：训练结果保存在 `yolov5/runs/train/exp$num$/weights/` 目录下。
- **使用方法**：将 `best.pt` 复制到 `yolov5/` 目录，并更新 `ros_detect.py` 中的权重路径以使用训练好的模型进行检测。

---

## 📚 使用说明
- **标注质量**：Grounding DINO 提供自动标注，但对于复杂场景可能需要通过 `better_anno.py` 进行优化以提高精度。
- **自定义**：调整 `labelling.py` 中的 `TEXT_PROMPT` 以匹配目标物体（如将“辣椒”改为“苹果”）。
- **图像大小**：确保 `resize.py` 输出 640x640 图像，这是 YOLOv5 的默认输入大小。如使用不同模型配置，需相应调整。
- **手动优化**：使用 `better_anno.py` 修正对齐不准或不准确的边界框，特别适用于小型或遮挡物体。
- **未来改进**：可集成预训练的检测或分割模型（如 SAM 或 YOLO-World）以简化 `better_anno.py` 的手动优化工作。

---

## 🔗 参考资料
- **Grounding DINO**：[IDEA-Research/GroundingDINO](https://github.com/IDEA-Research/GroundingDINO)
- **YOLOv5**：[ultralytics/yolov5](https://github.com/ultralytics/yolov5)
- **LabelImg**：[HumanSignal/labelImg](https://github.com/HumanSignal/labelImg)
