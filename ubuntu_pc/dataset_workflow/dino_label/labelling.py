import os
import shutil
import cv2
from groundingdino.util.inference import load_model, load_image, predict, annotate
from PIL import Image

# 加载 Grounding DINO 模型
model = load_model("home/sunx/code_2508/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py",
                   "/home/sunx/code_2508/GroundingDINO-alpha2/weights/groundingdino_swint_ogc.pth")

# model = load_model("/home/sunx/code_2508/GroundingDINO/groundingdino/config/GroundingDINO_SwinB_cfg.py",
#                    "/home/sunx/code_2508/GroundingDINO-alpha2/weights/groundingdino_swinb_cogcoor.pth")

# 定义配置
IMG_DIR = "/home/sunx/code_2508/auto_label/unlabel_img/captured/unlabel"  # 输入图像文件夹
OUTPUT_DIR = "/home/sunx/code_2508/auto_label/unlabel_img/captured/labelled"  # 输出 YOLO 数据集文件夹
TEXT_PROMPT = "red apple . green apple ."  # 检测提示
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25

# 类映射：red apple -> 0 (m), green apple -> 1 (im)
CLASS_MAP = {
    "green apple": 0,
    "red apple": 1
}
CLASS_NAMES = ['im', 'm']  # YOLO data.yaml 中的类名

# 创建 YOLO 数据集结构
os.makedirs(os.path.join(OUTPUT_DIR, "images/train"), exist_ok=True)
os.makedirs(os.path.join(OUTPUT_DIR, "labels/train"), exist_ok=True)

# 处理每个图像
for filename in os.listdir(IMG_DIR):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
        image_path = os.path.join(IMG_DIR, filename)

        # 加载图像
        image_source, image = load_image(image_path)

        # 预测
        boxes, logits, phrases = predict(
            model=model,
            image=image,
            caption=TEXT_PROMPT,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD
        )

        # 复制图像到 YOLO images/train
        train_image_path = os.path.join(OUTPUT_DIR, "images/train", filename)
        shutil.copy(image_path, train_image_path)

        # 获取图像尺寸（用于归一化确认，虽然 boxes 已归一化）
        height, width = image_source.shape[:2]

        # 生成标签文件
        label_filename = os.path.splitext(filename)[0] + ".txt"
        label_path = os.path.join(OUTPUT_DIR, "labels/train", label_filename)

        with open(label_path, "w") as f:
            for box, logit, phrase in zip(boxes, logits, phrases):
                phrase_lower = phrase.lower().strip()
                if phrase_lower in CLASS_MAP:
                    class_id = CLASS_MAP[phrase_lower]
                    cx, cy, w, h = box.tolist()  # boxes 是 cxcywh 归一化
                    f.write(f"{class_id} {cx} {cy} {w} {h}\n")

# 创建 data.yaml
data_yaml_path = os.path.join(OUTPUT_DIR, "data.yaml")
with open(data_yaml_path, "w") as f:
    f.write(f"""path: {os.path.abspath(OUTPUT_DIR)}
train: images/train
val: images/train
nc: {len(CLASS_NAMES)}
names: {CLASS_NAMES}
""")

print(f"YOLO 数据集生成完成，路径: {OUTPUT_DIR}")
print(f"data.yaml 已创建，类名: {CLASS_NAMES}")