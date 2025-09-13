import os
from PIL import Image

def convert_jpg_to_png(folder_path):
    # 遍历文件夹
    for filename in os.listdir(folder_path):
        if filename.lower().endswith(".jpg"):
            jpg_path = os.path.join(folder_path, filename)
            png_path = os.path.splitext(jpg_path)[0] + ".png"

            try:
                # 打开 JPG 并保存为 PNG
                img = Image.open(jpg_path).convert("RGB")
                img.save(png_path, "PNG")

                # 删除原 JPG
                os.remove(jpg_path)
                print(f"✅ 转换完成: {filename} -> {os.path.basename(png_path)}")

            except Exception as e:
                print(f"❌ 转换失败 {filename}: {e}")

if __name__ == "__main__":
    folder = "images"  # 修改为你的图片文件夹路径
    convert_jpg_to_png(folder)
