#!/usr/bin/env python3
import rospy
import math
import threading
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
from moveit_ctrl.srv import JointMoveitCtrlRequest as PiperMoveitCtrlRequest
from astra_camera.msg import LabelXYZ
import tf2_ros
import tf2_geometry_msgs

class FruitPickerDebug:
    def __init__(self):
        rospy.init_node('fruit_picker_debug')

        # 观察姿态（用于观测的end_pose）
        self.initial_pose = [0, 0.6755837736314219, 0, 0.7372832324188091]
        self.target_orie = self.initial_pose.copy()

        # 参数设置
        self.detect_duration = 1.0  # 观察时间（秒）
        self.pick_wait_duration = 2.0  # 每次夹取后等待时间（秒）
        self.dup_threshold = 0.1  # 两个果实的欧式距离小于 0.1m 则认定为重复

        # 状态变量
        self.joint_states = [0.0] * 8
        self.detected_fruits_base = []  # 储存base坐标下的果实位置

        # TF变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅
        rospy.Subscriber('/joint_states', JointState, self.joint_cb)
        rospy.Subscriber('/end_pose', PoseStamped, self.endpose_cb)
        rospy.Subscriber('/yolo/label_xyz', LabelXYZ, self.label_cb)

        # 服务客户端
        rospy.wait_for_service('/joint_moveit_ctrl_endpose')
        rospy.wait_for_service('/joint_moveit_ctrl_piper')
        self.moveit_end_client = rospy.ServiceProxy('/joint_moveit_ctrl_endpose', JointMoveitCtrl)
        self.moveit_piper_client = rospy.ServiceProxy('/joint_moveit_ctrl_piper', JointMoveitCtrl)
        self.moveit_gripper_client = rospy.ServiceProxy('/joint_moveit_ctrl_gripper', JointMoveitCtrl)

        # 初始位置移动
        input("按下回车将机械臂移动到初始观察位置 ...")
        initial_xyz1 = [-0.108463, -0.007421, 0.479865]
        initial_xyz2 = [0.108463, 0.007421, 0.479865]
        initial_xyz3 = [0.008463, 0.007421, 0.479865]
        self.move_and_detect(initial_xyz1)
        self.move_and_detect(initial_xyz2)
        self.move_and_detect(initial_xyz3)

        # 启动观察
        input("按下回车启用YOLO检测并开始观察 ...")
        rospy.set_param("/go_detect", True)
        rospy.set_param("/yolo/show_image", True)
        rospy.loginfo("YOLO 检测已启用，观察开始")

        rospy.spin()

    def joint_cb(self, msg):
        self.joint_states = list(msg.position)

    def endpose_cb(self, msg):
        ori = msg.pose.orientation
        self.target_orie = [ori.x, ori.y, ori.z, ori.w]

    def label_cb(self, msg):
        if msg.label != "mature":
            return
        fruit_position = [msg.xyz.x, msg.xyz.y, msg.xyz.z]
        base_pose = self.transform_to_base_link(fruit_position)
        if base_pose:
            self.detected_fruits_base.append(base_pose)

    def transform_to_base_link(self, xyz):
        try:
            pt = PointStamped()
            pt.header.frame_id = "camera_link"
            pt.header.stamp = rospy.Time(0)
            pt.point = Point(*xyz)
            self.tf_buffer.can_transform("base_link", "camera_link", rospy.Time(0), timeout=rospy.Duration(1.0))
            trans = self.tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(0))
            result = tf2_geometry_msgs.do_transform_point(pt, trans)
            return [result.point.x, result.point.y, result.point.z]
        except Exception as e:
            rospy.logerr(f"❌ TF转换失败: {e}")
            return None

    def move_and_detect(self, target_xyz):
        self.execute_moveit_motion(target_xyz + self.initial_pose)  # Move to the target pose
        rospy.sleep(1)  # Wait for the arm to stabilize

        # Start YOLO detection for 1 second at this pose
        rospy.set_param("/go_detect", True)
        rospy.set_param("/yolo/show_image", True)
        rospy.sleep(1)
        rospy.set_param("/go_detect", False)
        rospy.set_param("/yolo/show_image", False)

        rospy.loginfo(f"🍓 Finished detection at {target_xyz}, saving detected fruits...")
        self.remove_duplicates_and_store()

    def remove_duplicates_and_store(self):
        unique_fruits = self.remove_duplicates(self.detected_fruits_base)
        for pose in unique_fruits:
            self.detected_fruits_base.append(pose)

    def remove_duplicates(self, xyz_list):
        unique = []
        for p in xyz_list:
            if all(sum((a - b) ** 2 for a, b in zip(p, q)) ** 0.5 > self.dup_threshold for q in unique):
                unique.append(p)
        return unique

    def stop_detection_and_pick_fruits(self):
        # Stop YOLO detection after 3 detection rounds
        rospy.set_param("/go_detect", False)
        rospy.set_param("/yolo/show_image", False)
        rospy.loginfo("🛑 停止YOLO检测，开始果实采摘...")

        # Perform pick action on each unique fruit
        for pose in self.detected_fruits_base:
            self.pick_single_fruit(pose)
            rospy.sleep(self.pick_wait_duration)  # Wait before picking the next fruit

    def pick_single_fruit(self, base_pose):
        # 计算目标Yaw角
        dx, dy = base_pose[0], base_pose[1]
        target_yaw = math.atan2(dy, dx)
        target_joint1 = max(min(target_yaw, 2.618), -2.618)

        # 控制base关节旋转
        piper_req = PiperMoveitCtrlRequest()
        piper_req.joint_states = ([target_joint1] + self.joint_states[1:])[:6]
        piper_req.gripper = 0.0  # Open gripper before moving
        piper_req.max_velocity = 2.0
        piper_req.max_acceleration = 2.0
        try:
            self.moveit_piper_client(piper_req)  # Move base joint1
            rospy.loginfo("✅ Piper service调用成功，等待末端指令...")
            rospy.sleep(0.5)
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Piper服务失败: {e}")
            return

        # 末端到目标位置
        self.execute_moveit_motion(base_pose + self.target_orie)

        # 控制夹爪
        self.control_gripper(0.5)  # Close the gripper
        rospy.sleep(1)  # Wait for gripper action to complete

    def control_gripper(self, gripper_pos):
        try:
            gripper_req = JointMoveitCtrlRequest()
            gripper_req.joint_endpose = [gripper_pos]
            gripper_req.max_velocity = 2.0
            gripper_req.max_acceleration = 2.0
            self.moveit_gripper_client(gripper_req)
            rospy.loginfo(f"✅ Gripper control succeeded with position: {gripper_pos}")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Gripper control failed: {e}")

    def execute_moveit_motion(self, target_pose):
        try:
            req = JointMoveitCtrlRequest()
            req.joint_endpose = target_pose
            req.max_velocity = 2.0
            req.max_acceleration = 2.0
            resp = self.moveit_end_client(req)
            if resp.status:
                rospy.loginfo("✅ 到达目标位置")
            else:
                rospy.logwarn(f"⚠️ 运动失败, 错误码: {resp.error_code}")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ MoveIt服务调用失败: {e}")

if __name__ == "__main__":
    try:
        FruitPickerDebug()
    except rospy.ROSInterruptException:
        pass
