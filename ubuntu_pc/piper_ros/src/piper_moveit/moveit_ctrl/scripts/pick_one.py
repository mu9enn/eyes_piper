#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
from moveit_ctrl.srv import JointMoveitCtrlRequest as PiperMoveitCtrlRequest
from astra_camera.msg import LabelXYZ
import tf2_ros
import tf2_geometry_msgs
import threading
from std_srvs.srv import Empty

class FruitPickerDebug:
    def __init__(self):
        rospy.init_node('fruit_picker_debug')

        # 初始姿态（仅用于第一次移动）
        self.initial_pose = [0, 0.6755837736314219, 0, 0.7372832324188091]
        self.target_orie = self.initial_pose.copy()

        # 当前关节状态
        self.joint_states = [0.0] * 8
        rospy.Subscriber('/joint_states', JointState, self.joint_cb)

        # 订阅末端姿态
        rospy.Subscriber('/end_pose', PoseStamped, self.endpose_cb)

        # 初始化 TF 变换器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # YOLO检测坐标输出订阅
        rospy.Subscriber('/yolo/label_xyz', LabelXYZ, self.label_cb)

        # 服务客户端
        rospy.wait_for_service('/joint_moveit_ctrl_endpose')
        rospy.wait_for_service('/joint_moveit_ctrl_piper')
        self.moveit_end_client = rospy.ServiceProxy('/joint_moveit_ctrl_endpose', JointMoveitCtrl)
        self.moveit_piper_client = rospy.ServiceProxy('/joint_moveit_ctrl_piper', JointMoveitCtrl)

        # 已处理果实
        self.processed_fruits = []
        self.lock = threading.Lock()

        # 初始位置移动
        input("按下回车将机械臂移动到初始观察位置 ...")
        initial_xyz = [-0.108463, -0.007421, 0.479865]
        self.execute_moveit_motion(initial_xyz + self.initial_pose)

        input("按下回车启用检测（设置 rosparam /go_detect := true） ...")
        rospy.set_param("/go_detect", True)
        rospy.set_param("/yolo/show_image", True)
        rospy.loginfo("YOLO 检测已启用")
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
        if not self.is_new_fruit(fruit_position):
            return

        rospy.loginfo(f"\U0001F353 检测到新果实: {fruit_position}, 转换并执行运动 ...")
        base_pose = self.transform_to_base_link(fruit_position)
        if base_pose:
            with self.lock:
                self.processed_fruits.append(fruit_position)

            # 步骤1：计算 base joint1 转角 delta_yaw（仅简化为绕Z轴）
            import math
            dx, dy = base_pose[0], base_pose[1]
            target_yaw = math.atan2(dy, dx)
            # current_joint1 = self.joint_states[0] if len(self.joint_states) > 0 else 0.0
            # delta_joint1 = target_yaw
            target_joint1 = max(min(target_yaw, 2.618), -2.618)

            # 步骤2：调用 Piper 控制 base joint1 转动，并张开夹爪
            piper_req = PiperMoveitCtrlRequest()
            piper_req.joint_states = [target_joint1] + self.joint_states[1:6]
            piper_req.gripper = 0.035
            piper_req.max_velocity = 2.0
            piper_req.max_acceleration = 2.0
            try:
                self.moveit_piper_client(piper_req)
                rospy.loginfo("✅ Piper service调用成功，等待新末端姿态...")
                rospy.sleep(0.5)
            except rospy.ServiceException as e:
                rospy.logerr(f"❌ Piper服务失败: {e}")
                return

            # 步骤3：拼接并移动至目标
            self.execute_moveit_motion(base_pose + self.target_orie)

    def is_new_fruit(self, pos):
        threshold = 0.05
        for old in self.processed_fruits:
            dist = sum((a - b) ** 2 for a, b in zip(old, pos)) ** 0.5
            if dist < threshold:
                return False
        return True

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