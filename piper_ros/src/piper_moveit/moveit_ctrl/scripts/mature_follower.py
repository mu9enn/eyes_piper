#!/usr/bin/env python3
import rospy
import threading
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
import math


class FruitPicker:
    def __init__(self):
        rospy.init_node('fruit_picker_node')

        # 参数初始化
        self.collect_duration = rospy.get_param('~collect_duration', 0.5)  # 采集数据时长
        # self.zero_speed_timeout = rospy.get_param('~zero_speed_timeout', 2.0)  # 静止超时阈值
        self.ws_boundary = rospy.get_param('~workspace_boundary', [5.0, 3.14, 3.14])  # 工作空间边界 [radius, theta, phi]
        self.basket_pose = [0, 0, 0, 0, 0, 0, 1]  # 收集箱位置
        self.tool_offset = Point(0, 0, 0)

        # 初始化锁和数据存储
        self.lock = threading.Lock()
        self.xyz_buffer = []
        self.latest_endpose = None
        self.zero_start = None
        self.detected_fruits = {}
        self.pick_epoch = 0
        self.pick_epoch_renew = False
        self.ws_offset = []

        # ROS 话题订阅
        rospy.Subscriber('/tree_num', Int32, self.tree_num_cb)
        rospy.Subscriber('/pick_epoch', Int32, self.pick_epoch_cb)
        rospy.Subscriber('/yolo/label_xyz', String, self.label_xyz_cb)
        rospy.Subscriber('/end_pose', PoseStamped, self.endpose_cb)
        # rospy.Subscriber('/joint_states', JointState, self.jstates_cb)

        # ROS 发布
        self.go_detect_pub = rospy.Publisher('/go_detect', Bool, queue_size=1)
        self.ws_offset_pub = rospy.Publisher('/ws_offset', Point, queue_size=1)
        self.pick_epoch_pub = rospy.Publisher('/pick_epoch', Int32, queue_size=1)

        # 服务初始化
        rospy.wait_for_service('/joint_moveit_ctrl_endpose')
        self.moveit_client = rospy.ServiceProxy('/joint_moveit_ctrl_endpose', JointMoveitCtrl)
        # rospy.wait_for_service('/joint_moveit_ctrl_arm')
        # self.joint_client = rospy.ServiceProxy('/joint_moveit_ctrl_arm', JointMoveitCtrl)

        # 日志输出
        rospy.loginfo("Fruit Picker node started, waiting for tree_num...")

        # 主循环
        rospy.spin()

    def tree_num_cb(self, msg):
        """ 监听树编号变化，根据树的编号触发不同的动作 """
        tree_num = msg.data
        if tree_num == 0:
            rospy.loginfo("Robot is moving, no action taken.")
        elif tree_num == -1:
            self.move_to_collection_box()
        elif 1 <= tree_num <= 8:
            rospy.loginfo(f"Starting picking process for tree {tree_num}.")
            self.run_picking()
        else:
            rospy.logwarn(f"Invalid tree_num: {tree_num}. No action taken.")

    def pick_epoch_cb(self, msg):
        """ 监听采摘周期变化，触发不同的观测位姿动作 """
        if msg.data:
            self.pick_epoch_renew = True


    def label_xyz_cb(self, msg):
        """ 监听 YOLO 标签，检测到 mature 时采集果实坐标 """
        if msg.data.label == "mature":
            rospy.loginfo("Detected 'mature' fruit, starting collection.")
            with self.lock:
                self.xyz_buffer.append(msg.data.xyz)


    def endpose_cb(self, msg):
        """ 更新机械臂末端位置 """
        with self.lock:
            self.latest_endpose = msg.pose.position

    def arrive_target(self, target_pose=None, target_js=None):
        """ 检测机械臂是否运动到了目标位置 """
        timeout = 10  # 设置最大超时10秒
        start_time = rospy.Time.now()

        if target_pose:
            while rospy.Time.now() - start_time < rospy.Duration(timeout):
                if self.latest_endpose:
                    error = math.sqrt(
                        (self.latest_endpose.x - target_pose[0]) ** 2 +
                        (self.latest_endpose.y - target_pose[1]) ** 2 +
                        (self.latest_endpose.z - target_pose[2]) ** 2
                    )
                    if error < 0.05:  # 误差阈值为0.05米
                        return True
            return False
        if target_js:
            # 类似地，检查关节状态是否到达目标关节状态
            pass
        return False

    def move_to_collection_box(self):
        """ 移动到收集箱位置 """
        self.execute_moveit_motion(self.basket_pose)

    def move_to_observation_pose(self, epoch):
        """ 根据 pick_epoch 移动到不同的观测位姿 """
        if epoch == 1:
            self.target_pose = [-0.004573, -0.180059, 0.417223, -0.46349221409293045, 0.4619860789055821, 0.5456727185112626, 0.5234358744346881]
        elif epoch == 2:
            # self.target_pose = [3, 3, 2, 0, 0, 0, 1]  # 假设的观测位姿
            self.target_pose = [-0.004573, -0.180059, 0.417223, -0.46349221409293045, 0.4619860789055821, 0.5456727185112626, 0.5234358744346881]
        elif epoch == 3:
            # self.target_pose = [4, 4, 2, 0, 0, 0, 1]  # 假设的观测位姿
            self.target_pose = [-0.004573, -0.180059, 0.417223, -0.46349221409293045, 0.4619860789055821, 0.5456727185112626, 0.5234358744346881]
        else:
            rospy.logwarn(f"Invalid pick_epoch: {epoch}. No action taken.")
            return
        self.execute_moveit_motion(self.target_pose)

    def execute_moveit_motion(self, target_pose):
        """ 执行末端位置控制 """
        try:
            req = JointMoveitCtrlRequest()
            req.joint_endpose = target_pose
            req.max_velocity = 1.5
            req.max_acceleration = 1.5
            resp = self.moveit_client(req)
            if resp.status:
                rospy.loginfo("MoveIt motion executed successfully.")
            else:
                rospy.logwarn(f"MoveIt motion failed with error code {resp.error_code}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call MoveIt service: {e}")

    def collect_fruits(self):
        """ 收集所有 mature 标签的果实坐标储存到self.detected_fruits """

        def remove_duplicates(xyz_list):
            """ 基于欧氏距离去重果实坐标 """
            threshold = 0.1  # 设定欧氏距离阈值
            unique_fruits = {}
            for i, p1 in enumerate(xyz_list):
                for j, p2 in enumerate(xyz_list):
                    if i < j:
                        dist = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
                        if dist < threshold:
                            continue  # 跳过重复的果实
                unique_fruits[i] = [p1.x, p1.y, p1.z]
            return unique_fruits

        self.go_detect_pub.publish(True)
        rospy.sleep(0.5)  # 等待数据收集
        need_move = False

        # 去重算法：基于欧氏距离剔除重复果实
        filtered_fruits = remove_duplicates(self.xyz_buffer)

        # 计算果实的实际坐标
        if filtered_fruits:
            for idx, (x, y, z) in filtered_fruits.items():
                current_pose = self.latest_endpose

                # 计算目标末端位姿
                target_x = current_pose.x + x
                target_y = current_pose.y + y
                target_z = current_pose.z + z

                if self.check_workspace_boundary([target_x, target_y, target_z]):
                    self.detected_fruits[idx] = [target_x, target_y, target_z]

        else:
            rospy.loginfo("No mature fruits detected.")


    def pick_one_fruit(self, idx, x, y, z):
        """ 抓取果实动作 """
        self.target_pose = [x, y, z, -0.46349221409293045, 0.4619860789055821, 0.5456727185112626, 0.5234358744346881]  # 假设没有旋转
        self.execute_moveit_motion(self.target_pose)
        self.arrive_target(target_pose=self.target_pose)
        self.execute_moveit_motion(self.basket_pose)
        self.arrive_target(target_pose=self.basket_pose)


    def move_base(self):
        latest_epoch = self.pick_epoch
        self.pick_epoch_pub.pub(0)
        self.ws_offset_pub.pub(self.ws_offset)

        # 等待 pick_epoch_renew 为 True，再继续
        rate = rospy.Rate(10)
        while not self.pick_epoch_renew and not rospy.is_shutdown():
            rate.sleep()

        self.pick_epoch_renew = False
        self.pick_epoch = latest_epoch
        return

    def increment_epoch(self):
        """ 更新 pick_epoch """
        self.pick_epoch = (self.pick_epoch % 3) + 1
        rospy.loginfo(f"Pick epoch updated to {self.pick_epoch}")

    def check_workspace_boundary(self, target_pose):
        """ 检查目标位姿是否超出工作空间 """
        # 获取末端工具偏置（假设为 (0, 0, 0)）
        tool_offset = self.tool_offset

        # 计算目标位置相对于末端工具的偏移
        target_x = target_pose[0] - tool_offset.x
        target_y = target_pose[1] - tool_offset.y
        target_z = target_pose[2] - tool_offset.z

        # 转换为球坐标系
        r = math.sqrt(target_x ** 2 + target_y ** 2 + target_z ** 2)
        theta = math.acos(target_z / r) if r != 0 else 0
        phi = math.atan2(target_y, target_x)

        # 判断是否超出工作空间半径
        if r > self.ws_boundary[0]:
            # 计算超出部分的半径
            r_diff = r - self.ws_boundary[0]

            # 计算偏移量，确保底盘朝正确的方向移动
            dx = r_diff * math.sin(theta) * math.cos(phi)
            dy = r_diff * math.sin(theta) * math.sin(phi)
            dz = r_diff * math.cos(theta)

            # 创建偏移量点，并发布
            self.ws_offset.append(Point(dx, dy, dz))
            return False

        return True


    def run_picking(self):
        """一棵树的采摘流程"""
        while self.pick_epoch <= 3:
            self.picking_one_epoch()
            self.increment_epoch()


    def picking_one_epoch(self):
        """一个pick_epoch的采摘流程"""
        self.move_to_observation_pose(self.pick_epoch)
        self.arrive_target(target_pose=self.target_pose)
        self.collect_fruits()
        for idx, xyz in self.detected_fruits:
            x, y, z = xyz
            self.pick_one_fruit(idx, x, y, z)
        if self.ws_offset:
            self.move_base()
            self.picking_one_epoch()
        else:
            return

if __name__ == "__main__":
    try:
        FruitPicker()
    except rospy.ROSInterruptException:
        rospy.logerr("Fruit Picker node interrupted.")
