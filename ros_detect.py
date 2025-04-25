#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
from models.common import DetectMultiBackend
from utils.dataloaders import letterbox
from utils.general import non_max_suppression, scale_boxes
from utils.torch_utils import select_device
import numpy as np

class YoloDetector:
    def __init__(self):
        # Initialize ROS and utilities
        self.bridge = CvBridge()
        self.model = self.load_model()
        self.device = select_device('')  # Use '' for automatic device selection (CPU/GPU)
        self.model.to(self.device)

        # Subscribe to ROS topics
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)

        # Publishers for label and xyz
        self.label_pub = rospy.Publisher("/yolo/label", String, queue_size=10)
        self.xyz_pub = rospy.Publisher("/yolo/xyz", Point, queue_size=10)

        # Store incoming data
        self.color_image = None
        self.depth_image = None
        self.camera_info = None

        # YOLOv5 detection parameters
        self.conf_thres = 0.25  # Confidence threshold
        self.iou_thres = 0.45   # IoU threshold for NMS
        self.classes = None     # Filter by class (None = all classes)
        self.agnostic_nms = False  # Class-agnostic NMS
        self.max_det = 1000     # Maximum detections per image

    def load_model(self):
        # Load YOLOv5 model (default: yolov5s)
        weights = 'yolov5s.pt'  # Pre-trained weights file
        return DetectMultiBackend(weights, device=select_device(''))

    def color_callback(self, msg):
        # Convert ROS Image message to OpenCV format (BGR)
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image()
        except Exception as e:
            rospy.logerr(f"Error processing color image: {e}")

    def depth_callback(self, msg):
        # Convert ROS depth Image message to OpenCV format (16-bit unsigned)
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def info_callback(self, msg):
        # Store camera intrinsic parameters
        self.camera_info = msg

    def process_image(self):
        # Ensure all required data is available
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            return

        # Create a copy for visualization
        vis_image = self.color_image.copy()

        # Preprocess image for YOLOv5
        img = letterbox(self.color_image, 640, stride=32, auto=True)[0]  # Resize and pad
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device).float() / 255.0  # Normalize to [0, 1]
        if img.ndimension() == 3:
            img = img[None]  # Add batch dimension

        # Perform inference
        pred = self.model(img, augment=False, visualize=False)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes,
                                 self.agnostic_nms, max_det=self.max_det)

        # Process detection results
        for det in pred:  # Per image
            if len(det):
                # Rescale bounding boxes to original image size
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], self.color_image.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    label = self.model.names[int(cls)]  # Get object label
                    # Calculate bounding box center
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)

                    # Extract depth at center (convert from mm to meters)
                    depth = self.depth_image[center_y, center_x] / 1000.0
                    if depth > 0:  # Valid depth value
                        # Get camera intrinsics
                        fx = self.camera_info.K[0]  # Focal length x
                        fy = self.camera_info.K[4]  # Focal length y
                        cx = self.camera_info.K[2]  # Principal point x
                        cy = self.camera_info.K[5]  # Principal point y

                        # Compute 3D coordinates
                        x_3d = (center_x - cx) * depth / fx
                        y_3d = (center_y - cy) * depth / fy
                        z_3d = depth

                        # Log detection result
                        rospy.loginfo(f"Detected {label} at 3D position: ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f}) meters")

                        # Publish label
                        self.label_pub.publish(label)

                        # Publish xyz as geometry_msgs/Point
                        xyz_msg = Point()
                        xyz_msg.x = x_3d
                        xyz_msg.y = y_3d
                        xyz_msg.z = z_3d
                        self.xyz_pub.publish(xyz_msg)
                    else:
                        rospy.loginfo(f"Detected {label}, but depth is invalid")
                        self.label_pub.publish(label)  # Publish label even if depth is invalid

                    # Draw bounding box and label for visualization
                    x1, y1, x2, y2 = map(int, xyxy)
                    cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(vis_image, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the annotated image
        cv2.imshow("YOLOv5 Detections", vis_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("yolo_detector", anonymous=True)
    detector = YoloDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
