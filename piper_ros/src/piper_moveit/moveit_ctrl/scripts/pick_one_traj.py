#!/usr/bin/env python3
"""
完成一棵树下的 观察 - 采摘动作（不包含与其他模块的通信，对工作空间外的果实会解算超时）：
1. 执行本地轨迹，执行过程中进行YOLO检测
2. 将所有检测到的果实的 相机坐标 储存， 全部转换成 base_link 坐标后去重（基于欧式距离）
3. 针对每一个果实 执行采摘： 先控制 base（joint1）关节使机械臂朝向果实，再调用 endpose_ctrl 采摘
"""

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
import pickle
import moveit_commander

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
        rospy.loginfo('✅ start executing DEBUG')
        # input("按下回车将机械臂移动到初始观察位置 ...")

        traj = self.load_trajectory(filename="recorded_traj.pkl")
        self.move_and_detect(traj)

        # initial_xyz1 = [3.952030044646735e-05, 0.5059876845443003, -1.161216970816832, -3.061826620287533e-05, 0.6553283601500449, 9.559717424322598e-05]
        # initial_xyz2 = [-0.35239259419580926, 0.8002964971573558, -1.1612795629216544, 0.05370925566703078, 0.6552632675859501, 9.413290288408034e-05]
        # initial_xyz3 = [3.952030044646735e-05, 0.5059876845443003, -1.161216970816832, -3.061826620287533e-05, 0.6553283601500449, 9.559717424322598e-05]
        # self.move_and_detect(initial_xyz1)
        # self.move_and_detect(initial_xyz2)
        # self.move_and_detect(initial_xyz3)

        self.stop_detection_and_pick_fruits()

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

    def load_trajectory(self, filename="recorded_traj.pkl"):
        with open(filename, "rb") as f:
            traj = pickle.load(f)
        return traj

    def execute_trajectory(self, traj):
        moveit_commander.roscpp_initialize([])
        rospy.init_node("play_saved_traj", anonymous=True)
        group = moveit_commander.MoveGroupCommander("arm")  # 可改为 "piper"
        rospy.loginfo("Executing pre-built trajectory...")
        group.execute(traj, wait=True)
        moveit_commander.roscpp_shutdown()

    def move_and_detect(self, traj):
        # 加载轨迹并执行，启动YOLO检测

        rospy.set_param("/go_detect", True)
        rospy.set_param("/yolo/show_image", True)

        self.execute_trajectory(traj)

        # rospy.sleep(self.detect_duration)
        rospy.set_param("/go_detect", False)
        rospy.set_param("/yolo/show_image", False)

        rospy.loginfo(f"🍓 Finished detection, saving detected fruits...")
        self.remove_duplicates_and_store()

    def remove_duplicates_and_store(self):
        unique_fruits = self.remove_duplicates(self.detected_fruits_base)
        self.detected_fruits_base = unique_fruits

    def remove_duplicates(self, xyz_list):
        unique = []
        for p in xyz_list:
            if all(sum((a - b) ** 2 for a, b in zip(p, q)) ** 0.5 > self.dup_threshold for q in unique):
                unique.append(p)
        return unique

    def stop_detection_and_pick_fruits(self):
        rospy.set_param("/go_detect", False)
        rospy.set_param("/yolo/show_image", False)
        rospy.loginfo("🛑 停止YOLO检测，开始果实采摘...")

        for pose in self.detected_fruits_base:
            self.pick_single_fruit(pose)
            rospy.sleep(self.pick_wait_duration)

        rospy.loginfo("No fruit left, exiting the program.")
        rospy.signal_shutdown("No fruit left, exiting the program.")

    def pick_single_fruit(self, base_pose):
        dx, dy = base_pose[0], base_pose[1]
        target_yaw = math.atan2(dy, dx)
        target_joint1 = max(min(target_yaw, 2.618), -2.618)

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

        self.execute_moveit_motion(base_pose + self.target_orie)
        rospy.sleep(1)

        self.control_gripper(0.5)
        rospy.sleep(1)

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
