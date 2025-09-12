# 📊 Dataset Generation for YOLOv5 Using Lang-SAM

![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) ![Python](https://img.shields.io/badge/Python-3.11-blue.svg) ![YOLOv5](https://img.shields.io/badge/YOLOv5-v7.0-brightgreen.svg)

This directory provides a pipeline for generating synthetic training data for **YOLOv5** using **Lang-SAM** (Language-augmented Segment Anything Model). While traditional dataset annotation often relies on manual tools like [LabelImg](https://github.com/HumanSignal/labelImg), this approach automates object segmentation and annotation, enabling rapid dataset creation for fine-tuning YOLOv5 models. Although the generated dataset may have lower quality compared to manual annotations, it facilitates quick prototyping and integration with robotic systems, such as fruit-picking applications.

---

## 📖 Overview

The scripts in this directory automate the process of generating a synthetic dataset for YOLOv5 training by:
1. Segmenting target objects (e.g., peppers) from a small set of captured images using Lang-SAM.
2. Labeling objects based on attributes (e.g., color for ripeness).
3. Resizing images and backgrounds to YOLOv5-compatible dimensions (640x640).
4. Synthesizing new images by pasting segmented objects onto diverse backgrounds.
5. Generating bounding box annotations and splitting the dataset into training and validation sets.

This pipeline is designed for tasks requiring fast dataset creation, such as fine-tuning YOLOv5 for specific object detection tasks in robotic perception.

---

## 🛠 Prerequisites

- **Software**:
  - Ubuntu 20.04
  - Python 3.11 (for Lang-SAM)
  - Anaconda (for environment management)
  - [YOLOv5](https://github.com/ultralytics/yolov5) dependencies
  - [Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything)
- **Hardware**: PC with sufficient storage for images (4000+ background images recommended).
- **Directory Structure**:
  - `captured/`: Contains images of target objects (e.g., peppers).
  - `backgrounds/`: Contains diverse background images (4000+ recommended for variety).
  - `yolo_data/`: Output directory for generated dataset (images and annotations).

---

## 🚀 Dataset Generation Pipeline

### 1. Setup Environment and Dependencies

1. **Create and Activate Virtual Environment**:
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

### 2. Prepare Input Data
- **Captured Images**: Place images of target objects (e.g., peppers) in `yolo_workflow/dataset_workflow/captured/`.
- **Background Images**: Collect 4000+ diverse background images and place them in `yolo_workflow/dataset_workflow/backgrounds/`. Diversity (e.g., different lighting, textures) improves model robustness.

### 3. Run Dataset Generation Scripts
Ensure the environment is active and navigate to the dataset workflow directory:
```bash
conda activate langsam
cd ~/yolo_workflow/dataset_workflow
```

Run the following scripts in sequence:

1. **Segment and Label Objects**:
   ```bash
   python segment_label.py
   ```
   - **Function**: Uses Lang-SAM to segment objects (e.g., red and yellow peppers) from `captured/` images and labels them (e.g., red as `1` for mature, yellow as `0` for immature).
   - **Customization**: Modify `segment_label.py` for your specific task (e.g., different objects or labeling criteria).

2. **Resize Images**:
   ```bash
   python resize_pics.py
   ```
   - **Function**: Resizes segmented objects and background images to 640x640, the default input size for YOLOv5.

3. **Synthesize Images**:
   ```bash
   python synthesize_images.py
   ```
   - **Function**: Pastes segmented objects onto background images and generates bounding box annotations for the synthetic images.

4. **Verify Annotations**:
   ```bash
   python test_data.py
   ```
   - **Function**: Visualizes bounding boxes on generated images to check annotation accuracy.

5. **Split Dataset**:
   ```bash
   python train_val.py
   ```
   - **Function**: Splits the generated dataset into training and validation sets, stored in `yolo_data/images/train/` and `yolo_data/images/val/`.

---

## 🧠 Training YOLOv5 with Generated Data

### 1. Create Dataset Configuration
```bash
cd ~/yolo_workflow/yolov5/data
touch mydataset.yaml
```

### 2. Edit `mydataset.yaml`
Example configuration (adjust paths and class names for your task):
```yaml
# Train/val/test sets as 1) dir: path/to/imgs, 2) file: path/to/imgs.txt, or 3) list: [path/to/imgs1, path/to/imgs2, ..]
path: /home/sunx/code_proj/dataset_workflow/yolo_data  # Dataset root directory
train: images/train  # Training images (relative to 'path')
val: images/val      # Validation images (relative to 'path')
test:                # Test images (optional)

# Classes
nc: 2                # Number of classes
names: ['immature', 'mature']  # Class names
```

### 3. Run Training
```bash
cd ~/yolo_workflow/yolov5
python train.py --data mydataset.yaml --epochs 300 --weights '' --cfg yolov5s.yaml --batch-size 128
```
- **Output**: Training results are saved in `yolov5/runs/train/exp$num$/weights/`.
- **Usage**: Copy `best.pt` to `yolov5/` and update the weight path in `ros_detect.py` to use the trained model for detection.

---

## 📚 Usage Notes
- **Data Quality**: The synthetic dataset may have lower quality than manually annotated data. Verify bounding box accuracy using `test_data.py` and refine `segment_label.py` as needed.
- **Customization**: Modify `segment_label.py` to adapt to your target objects and labeling logic (e.g., different colors or attributes).
- **Background Diversity**: Use 4000+ background images with varied scenes to improve model generalization.
- **Integration**: The generated dataset is designed for rapid prototyping, enabling quick fine-tuning for tasks like fruit detection in robotic systems.

---

## 🔗 References
- **Lang-SAM**: [luca-medeiros/lang-segment-anything](https://github.com/luca-medeiros/lang-segment-anything)
- **YOLOv5**: [ultralytics/yolov5](https://github.com/ultralytics/yolov5)
- **LabelImg**: [HumanSignal/labelImg](https://github.com/HumanSignal/labelImg)