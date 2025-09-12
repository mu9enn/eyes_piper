#!/usr/bin/env python3
import os
import subprocess
import json
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs

class YoloDepthProcessor:
    def __init__(self):
        rospy.loginfo("Initializing YOLO RKNN detector node...")
        self.bridge = CvBridge()

        # Set dynamic library path for RKNN
        self.lib_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/lib'
        self.env = os.environ.copy()
        self.env['LD_LIBRARY_PATH'] = self.lib_path + ':' + self.env.get('LD_LIBRARY_PATH', '')

        # Paths for executable and model
        self.demo_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/rknn_yolov5_demo'
        self.model_path = '/home/orangepi/桌面/catkin_ros/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux/model/RK3588/yolov5.rknn'
        self.option = 'letterbox'
        self.temp_image_path = '/tmp/temp_image.jpg'

        # Image and camera info storage
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.image_ready = False

        # TF2 for coordinate transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS subscribers
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback, queue_size=1)
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)

        # ROS publisher for transformed coordinates
        self.m_xyz_pub = rospy.Publisher("/yolo/m_xyz", Point, queue_size=10)

        # Timer for processing at 10 FPS (0.1s)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.loginfo("YOLO RKNN detector is ready.")

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_ready = True  # Mark new image for processing
        except Exception as e:
            rospy.logerr(f"[color_callback] Error: {e}")

    def depth_callback(self, msg):
        try:
            # Assume depth image is 16-bit in mm
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr(f"[depth_callback] Error: {e}")

    def info_callback(self, msg):
        self.camera_info = msg

    def timer_callback(self, event):
        if self.image_ready and self.color_image is not None and self.depth_image is not None and self.camera_info is not None:
            self.process_image()
            self.image_ready = False  # Reset after processing

    def process_image(self):
        # Save RGB image to temporary file
        cv2.imwrite(self.temp_image_path, self.color_image)

        # Run YOLO detection
        cmd = [self.demo_path, self.model_path, self.temp_image_path, self.option]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True,
                env=self.env
            )

            # Parse JSON output
            output = result.stdout
            try:
                detections = json.loads(output)
            except json.JSONDecodeError as e:
                rospy.logerr(f"Failed to parse JSON output: {e}")
                rospy.logerr(f"Raw output: {output}")
                return

            # Process detections
            self.process_detections(detections)

        except subprocess.CalledProcessError as e:
            rospy.logerr("Failed to run the executable!")
            rospy.logerr(f"Error: {e.stderr}")

    def transform_to_base_link(self, xyz):
        try:
            pt = PointStamped()
            pt.header.frame_id = "camera_link"
            pt.header.stamp = rospy.Time(0)  # Use latest available transform
            pt.point = Point(*xyz)
            self.tf_buffer.can_transform("base_link", "camera_link", rospy.Time(0), timeout=rospy.Duration(1.0))
            trans = self.tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(0))
            result = tf2_geometry_msgs.do_transform_point(pt, trans)
            return [result.point.x, result.point.y, result.point.z]
        except Exception as e:
            rospy.logerr(f"TF transform failed: {e}")
            return None

    def process_detections(self, detections):
        h, w = self.depth_image.shape
        fx = self.camera_info.K[0]  # Focal length x
        fy = self.camera_info.K[4]  # Focal length y
        px = self.camera_info.K[2]  # Principal point x
        py = self.camera_info.K[5]  # Principal point y

        rospy.loginfo("=== Detection Results ===")
        for det in detections:
            class_name = det.get('class_name')
            score = det.get('score')
            bbox = det.get('bbox')  # [left, top, right, bottom]

            # Calculate bbox center
            cx = (bbox[0] + bbox[2]) / 2.0
            cy = (bbox[1] + bbox[3]) / 2.0

            # Check if center is within depth image bounds
            cx = int(round(cx))
            cy = int(round(cy))
            if not (0 <= cx < w and 0 <= cy < h):
                rospy.logwarn(f"Detected point out of bounds: ({cx}, {cy}) for {class_name}")
                continue

            # Get depth (in mm, convert to meters)
            depth = self.depth_image[cy, cx] / 1000.0
            if depth == 0.0 or np.isnan(depth):
                rospy.logwarn(f"Invalid depth at pixel ({cx}, {cy}) for {class_name}")
                continue

            # Calculate 3D coordinates in camera frame
            x_3d = (cx - px) * depth / fx
            y_3d = (cy - py) * depth / fy
            z_3d = depth

            # Print camera frame coordinates
            rospy.loginfo(f"Class: {class_name}, 3D Coordinates (camera_link): ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f})")

            # Transform to base_link for class 'm'
            if class_name == 'm':
                base_xyz = self.transform_to_base_link([x_3d, y_3d, z_3d])
                if base_xyz is not None:
                    # Print base_link coordinates
                    rospy.loginfo(f"Class: {class_name}, 3D Coordinates (base_link): ({base_xyz[0]:.2f}, {base_xyz[1]:.2f}, {base_xyz[2]:.2f})")

                    # Publish to /yolo/m_xyz
                    point_msg = Point()
                    point_msg.x = base_xyz[0]
                    point_msg.y = base_xyz[1]
                    point_msg.z = base_xyz[2]
                    self.m_xyz_pub.publish(point_msg)

if __name__ == '__main__':
    rospy.init_node('yolo_rknn_depth_processor', anonymous=True)
    processor = YoloDepthProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
