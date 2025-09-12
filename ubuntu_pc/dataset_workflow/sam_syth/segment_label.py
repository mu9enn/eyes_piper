import os
import numpy as np
from PIL import Image
from lang_sam import LangSAM
import cv2


def segment_and_label(input_dir, output_dir, text_prompt="bell pepper"):
    """
    Segment bell peppers from input images, crop to the minimum enclosing rectangle,
    perform color recognition, and save labeled segments with no extra blank areas.

    Args:
        input_dir (str): Directory containing input images.
        output_dir (str): Directory to save segmented and labeled images.
        text_prompt (str): Prompt for LangSAM to identify bell peppers.
    """
    model = LangSAM()
    image_extensions = ('.jpg', '.jpeg', '.png')
    image_files = [f for f in os.listdir(input_dir) if f.lower().endswith(image_extensions)]

    os.makedirs(output_dir, exist_ok=True)

    for image_file in image_files:
        image_path = os.path.join(input_dir, image_file)
        image_pil = Image.open(image_path).convert("RGB")
        width, height = image_pil.size
        results = model.predict([image_pil], [text_prompt])[0]

        combined_mask = np.zeros((height, width), dtype=np.uint8)
        for idx, mask in enumerate(results['masks']):
            # Convert mask to binary image and resize to match input image
            mask_np = (mask * 255).astype(np.uint8)
            mask_img = Image.fromarray(mask_np).resize(image_pil.size, Image.NEAREST)
            mask_np = np.array(mask_img) // 255
            combined_mask = np.logical_or(combined_mask, mask_np).astype(np.uint8)

            image_rgba = image_pil.convert("RGBA")
            image_array = np.array(image_rgba)
            output_array = np.zeros_like(image_array)
            output_array[combined_mask == 1] = image_array[combined_mask == 1]
            output_img = Image.fromarray(output_array, "RGBA")

            # Find the minimum enclosing rectangle
            coords = np.where(mask_np == 1)
            if len(coords[0]) == 0:
                continue  # Skip if no mask pixels
            y_min, y_max = coords[0].min(), coords[0].max()
            x_min, x_max = coords[1].min(), coords[1].max()

            # Crop the original image and mask to the bounding box
            bell_pepper = output_img.crop((x_min, y_min, x_max + 1, y_max + 1))
            mask_cropped = mask_img.crop((x_min, y_min, x_max + 1, y_max + 1))

            # Convert to NumPy arrays for color analysis
            bell_pepper_np = np.array(bell_pepper)
            mask_cropped_np = np.array(mask_cropped) // 255

            # Apply mask to cropped image
            bell_pepper_masked = bell_pepper_np * mask_cropped_np[:, :, np.newaxis]

            # Convert to HSV for color analysis
            hsv = cv2.cvtColor(bell_pepper_masked, cv2.COLOR_RGB2HSV)

            # Define color ranges for yellow and red
            yellow_lower = np.array([20, 100, 100])
            yellow_upper = np.array([30, 255, 255])
            red_lower1 = np.array([0, 100, 100])
            red_upper1 = np.array([10, 255, 255])
            red_lower2 = np.array([170, 100, 100])
            red_upper2 = np.array([180, 255, 255])

            # Create masks for yellow and red
            yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
            red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
            red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
            red_mask = red_mask1 + red_mask2

            # Calculate pixel counts within the mask
            yellow_pixels = np.sum((yellow_mask > 0) & (mask_cropped_np > 0))
            red_pixels = np.sum((red_mask > 0) & (mask_cropped_np > 0))
            total_pixels = np.sum(mask_cropped_np)

            # Assign label based on threshold (50% dominance)
            if yellow_pixels > red_pixels and yellow_pixels / total_pixels > 0.5:
                label = 0  # Yellow (immature)
            elif red_pixels > yellow_pixels and red_pixels / total_pixels > 0.5:
                label = 1  # Red (mature)
            else:
                print('Ambigulous, skipped.')  # Skip ambiguous cases
                label = 0

            # Save cropped bell pepper with label
            base_name = os.path.splitext(image_file)[0]
            output_filename = f"{label}_{base_name}.png"
            output_path = os.path.join(output_dir, output_filename)
            bell_pepper.save(output_path, quality=95)


if __name__ == "__main__":
    input_dir = "captured"
    output_dir = "segmented"
    segment_and_label(input_dir, output_dir)
