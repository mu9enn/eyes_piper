#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from astra_camera.srv import DetectObject, DetectObjectResponse
from astra_camera.msg import LabelXYZ
from std_msgs.msg import Bool
import cv2
import torch
import numpy as np

from models.common import DetectMultiBackend
from utils.dataloaders import letterbox
from utils.general import non_max_suppression, scale_boxes
from utils.torch_utils import select_device

class YoloDetector:
    def __init__(self):
        rospy.loginfo("Initializing YOLO detector node...")
        self.bridge = CvBridge()
        self.device = select_device('')  # 自动选择设备
        self.model = None

        # 相机信息
        self.color_image = None
        self.depth_image = None
        self.camera_info = None

        # 上一次检测结果
        self.last_label = ""
        self.last_position = Point()

        # YOLO配置
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.classes = None
        self.agnostic_nms = False
        self.max_det = 10

        # 订阅图像
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)

        # 发布检测结果
        self.label_xyz_pub = rospy.Publisher("/yolo/label_xyz", LabelXYZ, queue_size=10)

        # 服务回调
        self.detect_service = rospy.Service("detect_object", DetectObject, self.handle_detect_service)

        rospy.loginfo("Set /go_detect := true to enable detection.")
        rospy.loginfo("Set /yolo/show_image := true to visualize detection.")

        # 定时器，确保每秒5帧（0.2秒处理一次图像）
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

        self.image_ready = False  # 标记是否有新的图像需要处理

    def load_model(self):
        weights = '/home/sunx/code_proj/eyes_piper/piper_ros/src/ros_astra_camera/scripts/best.pt'
        self.model = DetectMultiBackend(weights, device=self.device).to(self.device)
        rospy.loginfo("🛑 YOLO model loaded successfully.")

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_ready = True  # 新图像准备好了
        except Exception as e:
            rospy.logerr(f"[color_callback] Error: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr(f"[depth_callback] Error: {e}")

    def info_callback(self, msg):
        self.camera_info = msg

    def timer_callback(self, event):
        # 定时器回调，每 0.2 秒触发一次
        if self.image_ready:  # 只有图像准备好时才进行处理
            self.process_image()
            self.image_ready = False  # 处理完毕后，重置标记

    def process_image(self):
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            return

        # 原图拷贝
        img0 = self.color_image.copy()
        img = letterbox(img0, 640, stride=32, auto=True)[0]
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device).float() / 255.0
        if img.ndimension() == 3:
            img = img[None]

        pred = self.model(img, augment=False, visualize=False)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres,
                                   self.classes, self.agnostic_nms, self.max_det)

        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    label = self.model.names[int(cls)]

                    # 计算中心点
                    cx = int((xyxy[0] + xyxy[2]) / 2)
                    cy = int((xyxy[1] + xyxy[3]) / 2)

                    # 深度边界检查
                    h, w = self.depth_image.shape
                    if not (0 <= cx < w and 0 <= cy < h):
                        rospy.logwarn(f"[WARN] Detected point out of bounds: ({cx},{cy})")
                        continue

                    depth = self.depth_image[cy, cx] / 1000.0
                    if depth == 0.0 or np.isnan(depth):
                        rospy.logwarn(f"[WARN] Invalid depth at pixel ({cx}, {cy})")
                        continue

                    # 相机内参解算
                    fx = self.camera_info.K[0]
                    fy = self.camera_info.K[4]
                    px = self.camera_info.K[2]
                    py = self.camera_info.K[5]

                    x3 = (cx - px) * depth / fx
                    y3 = (cy - py) * depth / fy
                    z3 = depth

                    # 更新并发布
                    self.last_label = label
                    self.last_position = Point(x3, y3, z3)

                    msg = LabelXYZ()
                    msg.label = label
                    msg.xyz = self.last_position
                    self.label_xyz_pub.publish(msg)

                    # 可视化可选
                    if rospy.get_param("/yolo/show_image", True):
                        x1, y1, x2, y2 = map(int, xyxy)
                        cv2.rectangle(img0, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(img0, f"{label} {conf:.2f}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if rospy.get_param("/yolo/show_image", True):
            cv2.imshow("YOLOv5 Detection", img0)
            cv2.waitKey(1)

    def handle_detect_service(self, req):
        resp = DetectObjectResponse()
        resp.label = self.last_label
        resp.position = self.last_position
        return resp

if __name__ == "__main__":
    rospy.init_node("yolo_detector_service")
    detector = YoloDetector()
    detector.load_model()
    rospy.loginfo("YOLO detector service is ready.")
    rospy.spin()
    cv2.destroyAllWindows()
