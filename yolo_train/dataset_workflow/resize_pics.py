import os
import cv2
import numpy as np

input_folder = 'segmented'
output_folder = 'segmented'

image_files = [f for f in os.listdir(input_folder) if f.endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]

for image_file in image_files:
    image_path = os.path.join(input_folder, image_file)
    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    if image is not None:
        if image.shape[2] == 4:
            h, w = image.shape[:2]
            new_width = 640
            new_height = int(new_width * h / w)  # 保持宽高比
            resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
            output_path = os.path.join(output_folder, image_file)
            cv2.imwrite(output_path, resized_image)

            print(f"已处理: {image_file}")
        else:
            print(f"跳过文件 {image_file}，因为它不是RGBA格式")
    else:
        print(f"无法读取文件 {image_file}")


input_folder = 'backgrounds'
output_folder = 'backgrounds'

image_files = [f for f in os.listdir(input_folder) if f.endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]

for image_file in image_files:
    image_path = os.path.join(input_folder, image_file)
    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    if image is not None:
        h, w = image.shape[:2]
        new_width = 640
        new_height = int(new_width * h / w)  # 保持宽高比
        resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        output_path = os.path.join(output_folder, image_file)
        cv2.imwrite(output_path, resized_image)

        print(f"已处理: {image_file}")
        print(f"跳过文件 {image_file}，因为它不是RGBA格式")
    else:
        print(f"无法读取文件 {image_file}")

print("所有图片已处理完成！")
