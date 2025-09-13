import os
import cv2
import numpy as np

# Define directories
IMAGES_DIR = "images/train"
LABELS_DIR = "labels/train"

# Class names for display
CLASS_NAMES = ['im', 'm']  # 0: green apple (im), 1: red apple (m)
COLORS = [(0, 255, 0), (0, 0, 255)]  # Green for 0, Red for 1

# Global variables for mouse clicks
click_count = 0
pt1 = None
pt2 = None
drawing = False
current_image = None
current_image_copy = None
img_name = None
label_path = None

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global click_count, pt1, pt2, drawing, current_image_copy

    if event == cv2.EVENT_LBUTTONDOWN:
        if click_count == 0:
            pt1 = (x, y)
            click_count = 1
            print(f"Left-top corner: {pt1}")
        elif click_count == 1:
            pt2 = (x, y)
            click_count = 2
            drawing = True
            # Draw the rectangle temporarily
            cv2.rectangle(current_image_copy, pt1, pt2, (255, 255, 0), 2)
            cv2.imshow("Image", current_image_copy)
            print(f"Right-bottom corner: {pt2}")

# Function to draw existing labels on the image
def draw_existing_labels(image, label_path):
    height, width = image.shape[:2]
    if os.path.exists(label_path):
        with open(label_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if len(parts) == 5:
                    class_id = int(parts[0])
                    cx = float(parts[1]) * width
                    cy = float(parts[2]) * height
                    w = float(parts[3]) * width
                    h = float(parts[4]) * height
                    x1 = int(cx - w / 2)
                    y1 = int(cy - h / 2)
                    x2 = int(cx + w / 2)
                    y2 = int(cy + h / 2)
                    color = COLORS[class_id]
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(image, CLASS_NAMES[class_id], (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    return image

# Main function
def main():
    global click_count, pt1, pt2, drawing, current_image, current_image_copy, img_name, label_path

    # Get list of images
    image_files = [f for f in os.listdir(IMAGES_DIR) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    for filename in image_files:
        image_path = os.path.join(IMAGES_DIR, filename)
        label_filename = os.path.splitext(filename)[0] + ".txt"
        label_path = os.path.join(LABELS_DIR, label_filename)

        # Load image
        current_image = cv2.imread(image_path)
        if current_image is None:
            print(f"Failed to load {filename}. Skipping...")
            continue

        print(f"Processing image: {filename}")

        # Annotation loop for this image
        while True:
            # Reset clicks
            click_count = 0
            pt1 = None
            pt2 = None
            drawing = False

            # Reload image and draw existing labels
            current_image_copy = current_image.copy()
            current_image_copy = draw_existing_labels(current_image_copy, label_path)

            # Show image
            cv2.imshow("Image", current_image_copy)
            cv2.setMouseCallback("Image", mouse_callback)

            # Wait for two clicks
            while click_count < 2:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    break

            if key == ord('q') or key == ord('Q'):
                print(f"Quitting annotation for {filename}")
                break

            if click_count == 2:
                # Get class from keyboard
                print("Enter class (0 for im/green, 1 for m/red): ")
                while True:
                    class_key = cv2.waitKey(0) & 0xFF
                    if class_key == ord('0'):
                        class_id = 0
                        break
                    elif class_key == ord('1'):
                        class_id = 1
                        break
                    else:
                        print("Invalid input. Enter 0 or 1.")

                # Wait for Enter to confirm
                print("Press Enter to confirm, or any other key to cancel.")
                confirm_key = cv2.waitKey(0) & 0xFF
                if confirm_key == 13:  # Enter key
                    # Calculate normalized YOLO format
                    height, width = current_image.shape[:2]
                    x1, y1 = min(pt1[0], pt2[0]), min(pt1[1], pt2[1])
                    x2, y2 = max(pt1[0], pt2[0]), max(pt1[1], pt2[1])
                    cx = ((x1 + x2) / 2) / width
                    cy = ((y1 + y2) / 2) / height
                    w = (x2 - x1) / width
                    h = (y2 - y1) / height

                    # Append to label file
                    with open(label_path, 'a') as f:
                        f.write(f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")

                    print(f"Added annotation: {class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
                else:
                    print("Annotation cancelled.")

            # After annotation, loop back to reload and redraw

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()