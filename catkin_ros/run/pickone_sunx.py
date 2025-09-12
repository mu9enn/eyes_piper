#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix


class FruitPicker:
    def __init__(self):
        rospy.init_node('fruit_picker', anonymous=True)
        rospy.loginfo("🚀 Initializing Fruit Picker node...")

        # Parameters
        self.detect_duration = 1.0  # Detection duration (seconds)
        self.pick_wait_duration = 2.0  # Wait time after picking (seconds)
        self.dup_threshold = 0.1  # Deduplication threshold (meters)
        # self.end_to_base_distance = 0.1358 + 0.10  # Distance from end-effector to gripper base (0.1358 + 0.10)
        self.end_to_base_distance = 0.1358  # Distance from end-effector to gripper base (0.1358 + 0.10)
        self.workspace_radius_i = 0.1000  # Inner workspace radius
        self.workspace_radius_o = 0.8000  # Outer workspace radius

        self.observation_poses = {
            1: [0.01, 0.01, -0.01, 0.01, 0.01, 0.01],
            # 2: [0.0, 0.5, -0.8, 0.0, 0.655, 0.0],
            # 3: [1.05, 1.7, -1.45, -1.28, 1.0, 1.65]
            2: [0.30, 0.15, -0.15, -0.01, 0.15, 0.01],
            3: [-0.30, 0.15, -0.15, -0.01, 0.15, 0.01]
        }

        # Basket placement pose (fixed endpose [x, y, z, qx, qy, qz, qw])
        self.safe_basket_pose = [0.01, 0.50, -0.50, -0.01, 0.01, 0.01]
        # self.basket_pose = [-1.20, 0.7, -1.0, -1.6, -1.2, 1.8]
        self.basket_pose = [0.0, 1.386, -1.694, 1.366, 1.203, -1.123]
        self.basket_yaw = 0.74  # Maximum 2.60, minimum -2.62

        self.joint_states = [0.0] * 6
        self.current_pose = None  # Store latest pose from /end_pose
        self.target_orie = [0.0, 0.6755837736314219, 0.0, 0.7372832324188091]  # Initial orientation
        self.detected_fruits_base = []  # List of fruit coordinates in base_link
        self.endpose_arrive = False  # Flag for YOLO detection control
        self.observe_num = 0  # Current observation position (1, 2, or 3)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.arrive_sub = rospy.Subscriber('/wheels/arrive', Int32, self.arrive_callback, queue_size=1)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback, queue_size=1)
        self.endpose_sub = rospy.Subscriber('/end_pose', PoseStamped, self.endpose_callback, queue_size=1)
        self.fruit_sub = rospy.Subscriber('/yolo/m_xyz', Point, self.fruit_callback, queue_size=1)

        self.picked_pub = rospy.Publisher('/yolo/picked', Int32, queue_size=1)

        rospy.wait_for_service('/joint_moveit_ctrl_endpose')
        rospy.wait_for_service('/joint_moveit_ctrl_piper')
        rospy.wait_for_service('/joint_moveit_ctrl_gripper')
        self.moveit_end_client = rospy.ServiceProxy('/joint_moveit_ctrl_endpose', JointMoveitCtrl)
        self.moveit_piper_client = rospy.ServiceProxy('/joint_moveit_ctrl_piper', JointMoveitCtrl)
        self.moveit_gripper_client = rospy.ServiceProxy('/joint_moveit_ctrl_gripper', JointMoveitCtrl)

        rospy.loginfo("🤖 Fruit Picker node initialized, waiting for /wheels/arrive signal...")

    def joint_callback(self, msg):
        self.joint_states = list(msg.position)[:6]

    def endpose_callback(self, msg):
        self.current_pose = msg.pose

    def fruit_callback(self, msg):
        # Store fruit coordinates in base_link if detection is active
        if self.endpose_arrive:
            fruit_position = [msg.x, msg.y, msg.z]
            if self.check_in_workspace(fruit_position):
                self.detected_fruits_base.append(fruit_position)
            else:
                rospy.logwarn(f"⚠️ Fruit at {fruit_position} is outside workspace, discarding.")

    def arrive_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("📍 Received /wheels/arrive signal, starting observation and picking...")
            self.start_observation_cycle()

    def check_endpose_arrival(self, target):
        """Check if the arm has reached the target pose or joint state, with 3-second timeout."""
        start_time = rospy.get_time()
        timeout = 5.0  # ~-second timeout
        threshold = 0.03  # ~ radians for joints, ~ meters for position

        while rospy.get_time() - start_time < timeout:
            if len(target) == 6:
                if len(self.joint_states) >= 6:
                    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(target, self.joint_states)))
                    if distance < threshold:
                        rospy.loginfo(f"✅ Joint state reached, distance: {distance:.4f}, time: {rospy.get_time() - start_time:.3f}")
                        return True
                else:
                    rospy.logwarn("⚠️ Insufficient joint states data")
            elif len(target) == 7:  # Endpose [x, y, z, qx, qy, qz, qw]
                if self.current_pose:
                    current_pos = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z]
                    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(target[:3], current_pos)))
                    if distance < threshold:
                        rospy.loginfo(f"✅ Endpose reached, distance: {distance:.4f}")
                        return True
                else:
                    rospy.logwarn("⚠️ No current pose data available")
            rospy.sleep(0.1)
        rospy.loginfo("⏱️ Timeout reached, assuming arrival")
        return True

    def check_in_workspace(self, pose):
        """Check if a pose is within the arm's workspace."""
        distance = math.sqrt((pose[0] - 0.091)**2 + pose[1]**2 + (pose[2] - 0.123)**2)
        return self.workspace_radius_i <= distance <= self.workspace_radius_o

    def deduplicate_fruits(self, fruit_list):
        """Remove duplicate fruit coordinates based on Euclidean distance."""
        unique = []
        for p in fruit_list:
            if all(math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q))) > self.dup_threshold for q in unique):
                unique.append(p)
        return unique

    def calculate_gripper_base_position(self, end_pose):
        """Calculate gripper_base position from end_pose and orientation."""
        rotation_matrix = quaternion_matrix(self.target_orie)
        gripper_base_direction_vector = rotation_matrix[:3, 2]
        offset = [self.end_to_base_distance * gripper_base_direction_vector[i] for i in range(3)]
        gripper_base_pose = [end_pose[i] - offset[i] for i in range(3)]
        gripper_base_pose[2] += 0.05  # Adjust for 果篮高度
        return gripper_base_pose

    def move_to_observation_pose(self, observe_num):
        """Move to the observation pose for the given observe_num."""
        if observe_num not in self.observation_poses:
            rospy.logerr(f"❌ Invalid observe_num: {observe_num}")
            return False

        target_joint = self.observation_poses[observe_num]
        piper_req = JointMoveitCtrlRequest()
        piper_req.joint_states = target_joint
        piper_req.gripper = 0.035
        piper_req.max_velocity = 2.0
        piper_req.max_acceleration = 2.0

        try:
            resp = self.moveit_piper_client(piper_req)
            if resp.status:
                rospy.loginfo(f"👁️ Moved to observation pose {observe_num}")
                return self.check_endpose_arrival(target_joint)
            else:
                rospy.logwarn(f"⚠️ Failed to move to observation pose {observe_num}, error: {resp.error_code}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Piper service failed: {e}")
            return False

    def execute_endpose_motion(self, target_pose):
        """Move to the target endpose."""
        req = JointMoveitCtrlRequest()
        req.joint_endpose = target_pose + self.target_orie
        req.max_velocity = 2.0
        req.max_acceleration = 2.0
        try:
            resp = self.moveit_end_client(req)
            if resp.status:
                rospy.loginfo(f"🎯 Moved to target endpose {target_pose}")
                return self.check_endpose_arrival(target_pose + self.target_orie)
            else:
                rospy.logwarn(f"⚠️ Failed to move to endpose {target_pose}, error: {resp.error_code}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ MoveIt service failed: {e}")
            return False

    def execute_safe_basket_motion(self):
        req = JointMoveitCtrlRequest()
        req.joint_states = self.safe_basket_pose
        req.gripper = 0.0
        req.max_velocity = 2.0
        req.max_acceleration = 2.0
        try:
            resp = self.moveit_piper_client(req)
            if resp.status:
                rospy.loginfo("Moved to safe basket joint states")
                return self.check_endpose_arrival(self.safe_basket_pose)
            else:
                rospy.logwarn(f"⚠️ Failed to move to basket states, error: {resp.error_code}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ MoveIt service failed: {e}")
            return False

    def execute_basket_motion(self):
        req = JointMoveitCtrlRequest()
        req.joint_states = self.basket_pose
        req.gripper = 0.0
        req.max_velocity = 2.0
        req.max_acceleration = 2.0
        try:
            resp = self.moveit_piper_client(req)
            if resp.status:
                rospy.loginfo("🧺 Moved to basket joint states")
                return self.check_endpose_arrival(self.basket_pose)
            else:
                rospy.logwarn(f"⚠️ Failed to move to basket states, error: {resp.error_code}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ MoveIt service failed: {e}")
            return False

    def control_gripper(self, gripper_pos):
        """Control the gripper (0.035 = open, 0.0 = close)."""
        req = JointMoveitCtrlRequest()
        req.gripper = gripper_pos
        req.max_velocity = 2.0
        req.max_acceleration = 2.0
        try:
            resp = self.moveit_gripper_client(req)
            if resp.status:
                rospy.loginfo(f"🦾 Gripper set to position: {gripper_pos}")
                return self.check_endpose_arrival(self.joint_states)
            else:
                rospy.logwarn(f"⚠️ Gripper control failed, error: {resp.error_code}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Gripper service failed: {e}")
            return False

    def pick_fruit(self, fruit_pose):
        """Execute the picking sequence for a single fruit."""
        if not self.execute_safe_basket_motion():
            rospy.logwarn("⚠️ Failed to move to safe basket position")
            return False

        # Step 1: Rotate base joint and open gripper
        dx, dy = fruit_pose[0], fruit_pose[1]
        target_yaw = math.atan2(dy, dx)
        target_joint1 = max(min(target_yaw, 2.618), -2.618)
        piper_req = JointMoveitCtrlRequest()
        piper_req.joint_states = ([target_joint1] + self.joint_states[1:])
        piper_req.gripper = 0.035
        piper_req.max_velocity = 2.0
        piper_req.max_acceleration = 2.0

        try:
            resp = self.moveit_piper_client(piper_req)
            if not resp.status:
                rospy.logwarn(f"⚠️ Failed to rotate base joint, error: {resp.error_code}")
                return False
            if not self.check_endpose_arrival([target_joint1] + self.joint_states[1:]):
                rospy.logwarn("⚠️ Base joint rotation not confirmed")
                return False
            # Update target_orie after base joint rotation
            if self.current_pose:
                self.target_orie = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                                   self.current_pose.orientation.z, self.current_pose.orientation.w]
            else:
                rospy.logwarn("No orientation data available, using previous target_orie")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Piper service failed: {e}")
            return False

        # Step 2: Move to fruit position
        gripper_base_pose = self.calculate_gripper_base_position(fruit_pose)
        if not self.check_in_workspace(gripper_base_pose):
            rospy.logwarn(f"⚠️ Gripper base position {gripper_base_pose} is outside workspace")
            return False

        # Move to a certain distance in front of the fruit
        # front_pose = [gripper_base_pose[0] - 0.1, gripper_base_pose[1], gripper_base_pose[2]]  # todo:找到x或者y加上距离偏置
        #
        # if self.check_in_workspace(front_pose):
        #     if not self.execute_endpose_motion(front_pose):
        #         rospy.logwarn(f"⚠️ Failed to move to fruit front position {gripper_base_pose}")
        #         return False

        if not self.execute_endpose_motion(gripper_base_pose):
            rospy.logwarn(f"⚠️ Failed to move to fruit position {gripper_base_pose}")
            return False

        # Step 3: Close gripper to pick fruit
        if not self.control_gripper(0.0):
            rospy.logwarn("⚠️ Failed to close gripper")
            return False

        return True

    def place_in_basket(self):
        """Execute the basket placement sequence."""
        # Step 1: Move to basket position
        if not self.execute_safe_basket_motion():
            rospy.logwarn("⚠️ Failed to move to safe basket position")
            return False
        if not self.execute_basket_motion():
            rospy.logwarn("⚠️ Failed to move to basket")
            return False

        # Step 2: Rotate base joint
        piper_req = JointMoveitCtrlRequest()
        piper_req.joint_states = ([self.basket_yaw] + self.joint_states[1:])
        piper_req.gripper = 0.0
        piper_req.max_velocity = 2.0
        piper_req.max_acceleration = 2.0
        try:
            resp = self.moveit_piper_client(piper_req)
            if not resp.status:
                rospy.logwarn("⚠️ Failed to rotate base for basket")
                return False
            if not self.check_endpose_arrival([self.basket_yaw] + self.joint_states[1:]):
                rospy.logwarn("⚠️ Base joint rotation for basket not confirmed")
                return False
            # Update target_orie after base joint rotation
            if self.current_pose:
                self.target_orie = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                                   self.current_pose.orientation.z, self.current_pose.orientation.w]
            else:
                rospy.logwarn("No orientation data available, using previous target_orie")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Piper service failed: {e}")
            return False

        # Step 3: Open gripper to release fruit
        if not self.control_gripper(0.035):
            rospy.logwarn("⚠️ Failed to open gripper for basket")
            return False

        return True

    def start_observation_cycle(self):
        """Execute the observation and picking cycle."""
        # Increment observe_num (reset to 1 if > 3)
        self.observe_num = rospy.get_param('/yolo/observe_num', 0) + 1
        if self.observe_num > 3:
            self.observe_num = 1
        rospy.set_param('/yolo/observe_num', self.observe_num)
        rospy.loginfo(f"🔍 Starting observation cycle {self.observe_num}")

        # Clear previous detections
        self.detected_fruits_base = []

        # Move to observation pose
        if not self.move_to_observation_pose(self.observe_num):
            rospy.logerr(f"❌ Failed to move to observation pose {self.observe_num}")
            self.handle_empty_detection()
            return

        # Collect detections for detect_duration
        self.endpose_arrive = True
        rospy.sleep(self.detect_duration)
        self.endpose_arrive = False

        # Deduplicate detected fruits
        self.detected_fruits_base = self.deduplicate_fruits(self.detected_fruits_base)
        rospy.loginfo(f"🍎 Detected {len(self.detected_fruits_base)} unique fruits: {self.detected_fruits_base}")

        # Process detected fruits
        if not self.detected_fruits_base:
            rospy.loginfo("🔄 No fruits detected, advancing to next cycle")
            self.handle_empty_detection()
            return

        # Pick and place fruits
        while self.detected_fruits_base:
            fruit_pose = self.detected_fruits_base.pop(0)
            if self.pick_fruit(fruit_pose):
                if self.place_in_basket():
                    rospy.loginfo(f"✅ Successfully picked and placed fruit at {fruit_pose}")
                    rospy.sleep(self.pick_wait_duration)
                else:
                    rospy.logwarn(f"⚠️ Failed to place fruit at {fruit_pose} in basket")
            else:
                rospy.logwarn(f"⚠️ Failed to pick fruit at {fruit_pose}")

        # Check if more observations are needed
        self.handle_empty_detection()

    def handle_empty_detection(self):
        """Handle case when no fruits are detected or picking is complete."""
        if self.observe_num < 3:
            rospy.loginfo(f"➡️ Advancing to observation cycle {self.observe_num + 1}")
            self.start_observation_cycle()
        else:
            rospy.loginfo("✅ All observation cycles complete, signaling completion")

            # Finally go to initial position
            if not self.execute_safe_basket_motion():
                rospy.logwarn("⚠️ Failed to move to safe basket position")

            rospy.set_param('/yolo/observe_num', 0)
            self.picked_pub.publish(Int32(data=1))
            rospy.loginfo("📤 Published /yolo/picked = 1")


if __name__ == '__main__':
    try:
        picker = FruitPicker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Shutting down")
