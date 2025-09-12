import os
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
import numpy as np


def read_annotations(annotation_path):
    """
    Read YOLO format annotations from a text file.

    Args:
        annotation_path (str): Path to the annotation text file.

    Returns:
        list: List of tuples (class_id, center_x, center_y, width, height).
    """
    annotations = []
    if os.path.exists(annotation_path):
        with open(annotation_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    class_id = int(parts[0])
                    center_x = float(parts[1])
                    center_y = float(parts[2])
                    width = float(parts[3])
                    height = float(parts[4])
                    annotations.append((class_id, center_x, center_y, width, height))
    return annotations


def draw_bounding_boxes(image, annotations, image_size):
    """
    Draw bounding boxes on the image based on YOLO annotations.

    Args:
        image (PIL.Image): Input image to draw on.
        annotations (list): List of (class_id, center_x, center_y, width, height).
        image_size (tuple): (width, height) of the image.

    Returns:
        PIL.Image: Image with bounding boxes drawn.
    """
    draw = ImageDraw.Draw(image)
    img_width, img_height = image_size

    for class_id, center_x, center_y, width, height in annotations:
        # Convert YOLO coordinates to pixel coordinates
        x_center = center_x * img_width
        y_center = center_y * img_height
        box_width = width * img_width
        box_height = height * img_height

        # Calculate top-left and bottom-right corners
        x1 = x_center - box_width / 2
        y1 = y_center - box_height / 2
        x2 = x_center + box_width / 2
        y2 = y_center + box_height / 2

        # Draw rectangle (red for class 0, green for class 1)
        color = 'red' if class_id == 0 else 'green'
        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)

        # Draw class label
        label = f"Class {class_id}"
        draw.text((x1, y1 - 10), label, fill=color)

    return image


def verify_yolo_dataset(image_dir, label_dir, image_filename):
    """
    Verify a single image and its annotations by drawing bounding boxes and displaying the result.

    Args:
        image_dir (str): Directory containing the image files.
        label_dir (str): Directory containing the annotation files.
        image_filename (str): Name of the image file to verify (e.g., 'background1_syn.png').
    """
    # Paths to image and annotation files
    image_path = os.path.join(image_dir, image_filename)
    base_name = os.path.splitext(image_filename)[0]
    annotation_path = os.path.join(label_dir, f"{base_name}.txt")

    # Check if files exist
    if not os.path.exists(image_path):
        print(f"Image file {image_path} does not exist.")
        return
    if not os.path.exists(annotation_path):
        print(f"Annotation file {annotation_path} does not exist.")
        return

    # Load image
    image = Image.open(image_path).convert("RGB")
    image_size = image.size

    # Read annotations
    annotations = read_annotations(annotation_path)
    if not annotations:
        print(f"No annotations found in {annotation_path}.")

    # Draw bounding boxes
    image_with_boxes = draw_bounding_boxes(image, annotations, image_size)

    # Display the image using Matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(image_with_boxes)
    plt.axis('off')
    plt.title(f"Verification of {image_filename}")
    plt.savefig(f"{base_name}.png")
    plt.show()


if __name__ == "__main__":
    # Example usage
    image_dir = "/yolo_data/images"
    label_dir = "/yolo_data/labels"
    # Specify the image file to verify (replace with your specific file)
    image_filename = "image_1_syn.png"  # Replace with the actual image name

    verify_yolo_dataset(image_dir, label_dir, image_filename)