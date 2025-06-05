import os
import shutil
import random

images_dir = 'yolo_data/images'
labels_dir = 'yolo_data/labels'
train_images_dir = os.path.join(images_dir, 'train')
val_images_dir = os.path.join(images_dir, 'val')
train_labels_dir = os.path.join(labels_dir, 'train')
val_labels_dir = os.path.join(labels_dir, 'val')

os.makedirs(train_images_dir, exist_ok=True)
os.makedirs(val_images_dir, exist_ok=True)
os.makedirs(train_labels_dir, exist_ok=True)
os.makedirs(val_labels_dir, exist_ok=True)

image_files = [f for f in os.listdir(images_dir) if f.endswith('.png')]

random.shuffle(image_files)

train_size = int(0.8 * len(image_files))  # 80% 作为训练集
train_files = image_files[:train_size]
val_files = image_files[train_size:]

for image_file in train_files:
    label_file = os.path.splitext(image_file)[0] + '.txt'  # 假设标签文件扩展名是 .txt
    shutil.move(os.path.join(images_dir, image_file), os.path.join(train_images_dir, image_file))
    shutil.move(os.path.join(labels_dir, label_file), os.path.join(train_labels_dir, label_file))

for image_file in val_files:
    label_file = os.path.splitext(image_file)[0] + '.txt'
    shutil.move(os.path.join(images_dir, image_file), os.path.join(val_images_dir, image_file))
    shutil.move(os.path.join(labels_dir, label_file), os.path.join(val_labels_dir, label_file))

print("Files have been split into train and val sets!")
