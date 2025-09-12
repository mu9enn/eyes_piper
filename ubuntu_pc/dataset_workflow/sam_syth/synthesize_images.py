import os
import random
import numpy as np
from PIL import Image, ImageOps
import cv2


def random_mask_pixels(image, occlusion_prob=0.8, min_area_ratio=0.2, max_area_ratio=0.5):
    """
    Randomly mask a continuous area of the image by setting alpha to 0, with a given occlusion probability.

    Args:
        image (PIL.Image): Input image in RGBA format.
        occlusion_prob (float): Probability of applying occlusion (default: 0.8).
        min_area_ratio (float): Minimum ratio of the image area to occlude (default: 0.2).
        max_area_ratio (float): Maximum ratio of the image area to occlude (default: 0.5).

    Returns:
        PIL.Image: Image with a randomly masked continuous area (if occlusion occurs).
    """
    # Convert image to numpy array
    image_np = np.array(image)
    height, width = image_np.shape[:2]
    total_area = height * width

    # Decide whether to apply occlusion
    if np.random.random() > occlusion_prob:
        return Image.fromarray(image_np)  # No occlusion, return original image

    # Calculate the occlusion area
    area_ratio = np.random.uniform(min_area_ratio, max_area_ratio)
    occlusion_area = int(total_area * area_ratio)

    # Calculate dimensions of the occlusion rectangle
    # Assume a square-like region for simplicity (aspect ratio close to 1)
    side_length = int(np.sqrt(occlusion_area))
    occlude_height = min(side_length, height)
    occlude_width = min(int(occlusion_area / occlude_height), width)

    # Choose a random starting point for the occlusion
    max_start_y = height - occlude_height
    max_start_x = width - occlude_width
    start_y = np.random.randint(0, max_start_y + 1)
    start_x = np.random.randint(0, max_start_x + 1)

    # Apply occlusion by setting alpha to 0
    image_np[start_y:start_y + occlude_height, start_x:start_x + occlude_width, 3] = 0

    return Image.fromarray(image_np)


def random_augment(image):
    """
    Apply random scaling, rotation, and pixel masking to an image.

    Args:
        image (PIL.Image): Input image in RGBA format.

    Returns:
        PIL.Image: Augmented image with scaling, rotation, and pixel masking.
    """
    # Random scaling between 0.5x and 0.8x
    scale = random.uniform(0.1, 0.75)
    new_size = (int(image.width * scale), int(image.height * scale))
    image = image.resize(new_size, Image.LANCZOS)

    # Random rotation between -30 and 30 degrees
    angle = random.uniform(-180, 180)
    image = image.rotate(angle, expand=True, fillcolor=(0, 0, 0, 0))  # Transparent background

    # Apply random pixel masking
    image = random_mask_pixels(image)

    return image

def paste_on_background(background, bell_pepper, position, occupied_boxes):
    """
    Paste a bell pepper image onto a background at a specified position, avoiding overlap.

    Args:
        background (PIL.Image): Background image (RGBA).
        bell_pepper (PIL.Image): Bell pepper image (RGBA).
        position (tuple): (x, y) coordinates to paste the bell pepper.
        occupied_boxes (list): List of (x1, y1, x2, y2) bounding boxes of previously pasted peppers.

    Returns:
        PIL.Image: Background with bell pepper pasted.
        tuple: (x1, y1, x2, y2) bounding box of the pasted pepper, or None if not pasted.
    """
    # Get the tight bounding box of the bell pepper
    bell_pepper_np = np.array(bell_pepper)
    alpha = bell_pepper_np[:, :, 3]
    coords = np.where(alpha > 0)
    if len(coords[0]) == 0:
        return background, None

    y_min, y_max = coords[0].min(), coords[0].max()
    x_min, x_max = coords[1].min(), coords[1].max()

    # Calculate the bounding box in the background coordinates
    x, y = position
    x1 = x + x_min
    y1 = y + y_min
    x2 = x + x_max
    y2 = y + y_max

    # Check for overlap with existing boxes
    for ox1, oy1, ox2, oy2 in occupied_boxes:
        if not (x2 < ox1 or x1 > ox2 or y2 < oy1 or y1 > oy2):
            return background, None  # Overlap detected, skip pasting

    # Paste the bell pepper
    mask = bell_pepper.split()[3] if bell_pepper.mode == 'RGBA' else None
    background.paste(bell_pepper, position, mask)

    return background, (x1, y1, x2, y2)


def calculate_bounding_box(position, bell_pepper, image):
    """
    Calculate YOLO-format bounding box coordinates based on opaque pixels.

    Args:
        position (tuple): (x, y) top-left corner of the bell pepper.
        bell_pepper (PIL.Image): Bell pepper image (RGBA).
        image (PIL.Image): Background image.

    Returns:
        tuple: (center_x, center_y, width, height) in YOLO format, or None if no opaque pixels.
    """
    # Convert bell pepper to NumPy array and extract alpha channel
    bell_pepper_np = np.array(bell_pepper)
    alpha = bell_pepper_np[:, :, 3]

    # Find coordinates of opaque pixels
    coords = np.where(alpha > 0)
    if len(coords[0]) == 0:
        return None

    # Calculate bounding box in bell pepper image
    y_min, y_max = coords[0].min(), coords[0].max()
    x_min, x_max = coords[1].min(), coords[1].max()

    # Convert to background coordinates
    x, y = position
    x1 = x + x_min
    y1 = y + y_min
    x2 = x + x_max
    y2 = y + y_max

    # Calculate YOLO coordinates
    img_w, img_h = image.size
    w = x2 - x1
    h = y2 - y1
    center_x = (x1 + w / 2) / img_w
    center_y = (y1 + h / 2) / img_h
    width = w / img_w
    height = h / img_h

    return (center_x, center_y, width, height)


def synthesize_images(bell_pepper_dir, background_dir, output_image_dir, output_label_dir):
    """
    Synthesize images by pasting bell peppers onto backgrounds and generate YOLO annotations.

    Args:
        bell_pepper_dir (str): Directory with segmented bell pepper PNG images.
        background_dir (str): Directory with background images.
        output_image_dir (str): Directory to save synthesized images.
        output_label_dir (str): Directory to save YOLO annotation files.
    """
    bell_pepper_files = [f for f in os.listdir(bell_pepper_dir) if f.endswith('.png')]
    background_files = [f for f in os.listdir(background_dir) if f.endswith('.png')]

    os.makedirs(output_image_dir, exist_ok=True)
    os.makedirs(output_label_dir, exist_ok=True)

    for bg_file in background_files:
        bg_path = os.path.join(background_dir, bg_file)
        try:
            background = Image.open(bg_path).convert("RGBA")
        except OSError as e:
            print(f"Skipping {bg_path}: {e}")
            continue

        # Randomly select 1 to 3 peppers
        num_peppers = random.randint(1, 3)
        selected_peppers = random.sample(bell_pepper_files, min(num_peppers, len(bell_pepper_files)))

        annotations = []
        occupied_boxes = []

        for pepper_file in selected_peppers:
            pepper_path = os.path.join(bell_pepper_dir, pepper_file)
            pepper = Image.open(pepper_path).convert("RGBA")

            # Augment the bell pepper (includes scaling, rotation, and masking)
            pepper = random_augment(pepper)

            # Try to find a non-overlapping position (up to 10 attempts)
            max_attempts = 10
            for _ in range(max_attempts):
                # Ensure bell pepper fits within background
                max_x = max(0, background.width - pepper.width)
                max_y = max(0, background.height - pepper.height)
                if max_x <= 0 or max_y <= 0:
                    break  # Skip if too large

                # Random position
                position = (random.randint(0, max_x), random.randint(0, max_y))

                # Attempt to paste the bell pepper
                background, bbox = paste_on_background(background, pepper, position, occupied_boxes)
                if bbox:
                    # Successful paste, update occupied boxes and calculate YOLO bbox
                    occupied_boxes.append(bbox)
                    class_id = int(pepper_file.split('_')[0])
                    yolo_bbox = calculate_bounding_box(position, pepper, background)
                    if yolo_bbox:
                        annotations.append(f"{class_id} {yolo_bbox[0]} {yolo_bbox[1]} {yolo_bbox[2]} {yolo_bbox[3]}")
                    break

        # Save synthesized image
        base_name = os.path.splitext(bg_file)[0]
        output_image_path = os.path.join(output_image_dir, f"{base_name}_syn.png")
        background.save(output_image_path)

        # Save annotation file
        output_label_path = os.path.join(output_label_dir, f"{base_name}_syn.txt")
        with open(output_label_path, 'w') as f:
            f.write('\n'.join(annotations))


if __name__ == "__main__":
    bell_pepper_dir = "segmented"
    background_dir = "backgrounds"
    output_image_dir = "yolo_data/images"
    output_label_dir = "yolo_data/labels"
    synthesize_images(bell_pepper_dir, background_dir, output_image_dir, output_label_dir)