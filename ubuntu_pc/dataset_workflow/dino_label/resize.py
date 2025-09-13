import os
from PIL import Image
import glob


def resize_images(input_dir, output_dir, target_size=(640, 640)):
    """
    调整指定目录下所有图片的尺寸

    参数:
        input_dir: 输入图片所在目录
        output_dir: 处理后图片的保存目录
        target_size: 目标尺寸，默认为(640, 640)
    """
    # 创建输出目录（如果不存在）
    os.makedirs(output_dir, exist_ok=True)

    # 支持的图片格式
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.gif']
    image_files = []

    # 收集所有图片文件
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(input_dir, ext)))

    if not image_files:
        print(f"在 {input_dir} 中未找到任何图片文件")
        return

    # 处理每张图片
    for img_path in image_files:
        try:
            # 打开图片
            with Image.open(img_path) as img:
                # 调整尺寸
                resized_img = img.resize(target_size)

                # 获取文件名
                filename = os.path.basename(img_path)

                # 保存处理后的图片
                output_path = os.path.join(output_dir, filename)
                resized_img.save(output_path)

                print(f"已处理: {filename}")
        except Exception as e:
            print(f"处理 {img_path} 时出错: {str(e)}")


if __name__ == "__main__":
    # 定义输入和输出目录
    directories = [
        ('images/train', 'images/train'),
        ('images/val', 'images/val')
    ]

    # 处理每个目录
    for input_dir, output_dir in directories:
        print(f"开始处理 {input_dir}...")
        resize_images(input_dir, output_dir)
        print(f"{input_dir} 处理完成\n")

    print("所有图片处理完毕！")
