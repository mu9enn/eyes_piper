import os
import subprocess
import json

# Set dynamic library path
lib_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/lib'
env = os.environ.copy()
env['LD_LIBRARY_PATH'] = lib_path + ':' + env.get('LD_LIBRARY_PATH', '')

# Path to the modified executable and model
demo_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/rknn_yolov5_demo'
model_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/model/RK3588/yolov5.rknn'
image_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/model/bus.jpg'
option = 'letterbox'

# Command to run the executable
cmd = [demo_path, model_path, image_path, option]

try:
    # Run the executable and capture output
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        check=True,
        env=env
    )

    # Parse the JSON output from the executable
    output = result.stdout
    try:
        detections = json.loads(output)
    except json.JSONDecodeError as e:
        print(f"Failed to parse output as JSON: {e}")
        print(f"Raw output: {output}")
        raise

    # Store and print detection results
    print("=== Detection Results ===")
    for det in detections:
        class_name = det.get('class_name')
        score = det.get('score')
        bbox = det.get('bbox')  # [left, top, right, bottom]
        print(f"Class: {class_name}, Score: {score:.2f}, BBox: {bbox}")

    # this should be like:
    # === Detection Results ===
    # Class: person, Score: 0.85, BBox: [100, 150, 200, 250]
    # Class: car, Score: 0.90, BBox: [300, 350, 400, 450]

except subprocess.CalledProcessError as e:
    print("Failed to run the executable!")
    print(f"Error: {e.stderr}")
    raise