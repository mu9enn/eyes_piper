# 果实采摘 ROS 节点文档

## 概述
`fruit_picker.py` 脚本实现了一个基于 ROS（机器人操作系统）的自主果实采摘系统，用于控制机械臂。该系统与基于 YOLO 的果实检测流水线集成，与移动底盘通信，控制机械臂进行观察、检测、采摘和将果实放置到篮子中。代码设计模块化、健壮，遵循工程最佳实践，能够处理如空检测和运动失败等边缘情况。

## ROS 接口

### 话题
- **订阅者**：
  - **/wheels/arrive** (`std_msgs/Int32`)：
    - 描述：接收来自移动底盘的信号，表示底盘运动完成。
    - 触发：值为 `1` 时启动观察和采摘循环。
    - 使用：通过 `arrive_callback` 回调启动工作流程。
  - **/joint_states** (`sensor_msgs/JointState`)：
    - 描述：提供机械臂当前关节角度（限制为 6 个关节）。
    - 使用：在 `joint_callback` 中更新 `self.joint_states`，用于到达检查。
  - **/end_pose** (`geometry_msgs/PoseStamped`)：
    - 描述：提供末端执行器的当前位姿（位置和方向）。
    - 使用：在 `endpose_callback` 中更新 `self.current_pose`，用于位置和方向跟踪。
  - **/yolo/m_xyz** (`geometry_msgs/Point`)：
    - 描述：从独立的 YOLO 检测节点接收检测到的果实（类别 `m`）在 `base_link` 坐标系中的 3D 坐标。
    - 使用：`fruit_callback` 回调在观察期间存储有效坐标。

- **发布者**：
  - **/yolo/picked** (`std_msgs/Int32`)：
    - 描述：发布信号，表示所有果实已采摘或三次观察循环后无果实剩余。
    - 消息：完成时发布 `1`。
    - 使用：标记一棵树的采摘过程结束。

### 参数
- **/yolo/observe_num**（整数，默认：0）：
  - 描述：跟踪当前观察位置（1、2 或 3）。
  - 使用：每次观察循环后递增，完成后重置为 0。

### 服务
- **/joint_moveit_ctrl_endpose** (`moveit_ctrl/JointMoveitCtrl`)：
  - 描述：将机械臂移动到指定的末端位姿（`[x, y, z, qx, qy, qz, qw]`）。
  - 使用：在 `execute_endpose_motion` 和 `execute_basket_motion` 中用于采摘和放置果篮。
- **/joint_moveit_ctrl_piper** (`moveit_ctrl/JointMoveitCtrl`)：
  - 描述：控制关节状态（如基座关节旋转）和夹爪动作。
  - 使用：在 `move_to_observation_pose`、`pick_fruit`、`place_in_basket` 和 `control_gripper` 中用于关节运动和夹爪控制。

## 系统参数
- **检测时长**：`self.detect_duration = 1.0` 秒（收集 YOLO 检测结果的时间）。
- **采摘等待时长**：`self.pick_wait_duration = 3.0` 秒（每次采摘后的等待时间）。
- **去重阈值**：`self.dup_threshold = 0.1` 米（果实去重的欧几里得距离阈值）。
- **末端到基座距离**：`self.end_to_base_distance = 0.2358` 米（末端执行器到夹爪基座的距离，用于工作空间检查）。
- **工作空间半径**：
  - 内半径：`self.workspace_radius_i = 0.1600` 米。
  - 外半径：`self.workspace_radius_o = 0.8000` 米。
- **观察位姿**：三个预定义的观察关节状态（6 元素列表，来源于 `/joint_states_single`）。
- **果篮位姿**：固定末端位姿 `[0.078499, 0.000509, 0.579909, 0.05646535193719382, 0.7336228643323106, 0.04860776277406437, 0.6754601708151009]`（位置 + 四元数方向）。
- **果篮偏航角**：`self.basket_yaw = 2.58` 弧度（果篮对齐的基座关节旋转，约束在 [-2.62, 2.60] 弧度）。

## 工作流程
`FruitPicker` 类协调以下工作流程，完成单棵树的果实采摘：

1. **初始化**：
   - 初始化 ROS 节点、订阅者、发布者和服务客户端。
   - 设置参数和状态变量（`self.joint_states`、`self.current_pose`、`self.detected_fruits_base`、`self.endpose_arrive`、`self.observe_num`）。
   - 配置 TF2 用于坐标转换。

2. **等待底盘信号**：
   - 监听 `/wheels/arrive` 话题，值为 `1` 时通过 `arrive_callback` 触发观察和采摘循环。

3. **观察循环**（`start_observation_cycle`）：
   - 递增 `/yolo/observe_num`（若大于 3 则重置为 1）。
   - 清空之前的果实检测结果。
   - 使用 `move_to_observation_pose` 移动到对应 `observe_num`（1、2 或 3）的观察位姿。
   - 设置 `self.endpose_arrive = True`，从 `/yolo/m_xyz` 收集 1 秒的 YOLO 检测结果。
   - 设置 `self.endpose_arrive = False` 停止检测。
   - 使用 `check_in_workspace` 过滤工作空间内的果实，并通过 `deduplicate_fruits` 去重。
   - 若无果实检测到，进入 `handle_empty_detection`。

4. **采摘与放置**：
   - 对 `self.detected_fruits_base` 中的每个去重果实坐标：
     - 执行 `pick_fruit`：
       1. 旋转基座关节对齐果实并打开夹爪（`gripper_pos = 0.035`）。
       2. 旋转后根据 `/end_pose` 更新 `self.target_orie`。
       3. 若果实在工作空间内，使用 `execute_endpose_motion` 移动到果实位置。
       4. 关闭夹爪（`gripper_pos = 0.0`）采摘果实。
     - 执行 `place_in_basket`：
       1. 使用 `execute_basket_motion` 移动到固定果篮位姿。
       2. 将基座关节旋转到 `self.basket_yaw` 并打开夹爪。
       3. 旋转后更新 `self.target_orie`。
       4. 打开夹爪释放果实。
     - 每次采摘后等待 3 秒（`pick_wait_duration`）。
   - 若采摘或放置失败，记录警告并继续处理下一个果实。

5. **处理空检测**（`handle_empty_detection`）：
   - 若 `observe_num < 3`，递增 `observe_num` 并开始新的观察循环。
   - 若 `observe_num == 3`，重置 `/yolo/observe_num` 为 0 并向 `/yolo/picked` 发布 `1`。

## 关键函数
- **check_endpose_arrival(target)**：
  - 检查机械臂是否到达目标（关节状态或末端位姿）。
  - 关节状态（6 元素）：与 `self.joint_states` 比较，使用欧几里得距离（阈值：0.01 弧度）。
  - 末端位姿（7 元素）：与 `self.current_pose.position` 的位置比较（阈值：0.01 米）。
  - 若达到阈值或 3 秒超时，返回 `True`。
- **check_in_workspace(pose)**：
  - 验证 3D 位姿是否在工作空间内（内半径：0.16 米，外半径：0.8 米，考虑基座偏移）。
  - 若在范围内，返回 `True`。
- **deduplicate_fruits(fruit_list)**：
  - 使用 0.1 米欧几里得距离阈值去除重复果实坐标。
  - 返回唯一坐标列表。
- **calculate_gripper_base_position(end_pose)**：
  - 根据末端位姿和方向计算夹爪基座位置，调整 0.2358 米偏移。
- **move_to_observation_pose(observe_num)**：
  - 使用 `/joint_moveit_ctrl_piper` 移动到三个预定义观察关节状态之一。
- **execute_endpose_motion(target_pose)**：
  - 使用 `/joint_moveit_ctrl_endpose` 移动到目标末端位置，保持当前方向。
- **execute_basket_motion()**：
  - 使用 `/joint_moveit_ctrl_endpose` 移动到固定果篮位姿。
- **control_gripper(gripper_pos)**：
  - 使用 `/joint_moveit_ctrl_piper` 设置夹爪状态（0.035 = 打开，0.0 = 关闭）。
- **pick_fruit(fruit_pose)**：
  - 执行采摘序列：旋转基座、移动到果实、关闭夹爪。
- **place_in_basket()**：
  - 执行果篮放置序列：移动到果篮、旋转基座、打开夹爪。
- **start_observation_cycle()**：
  - 管理观察和采摘循环，包括去重和观察位姿切换。
- **handle_empty_detection()**：
  - 推进到下一个观察循环或在完成时发送信号。

## 假设与约束
- **YOLO 检测**：独立节点向 `/yolo/m_xyz` 发布 `base_link` 坐标系中的果实坐标。
- **TF 转换**：假设 `camera_link` 到 `base_link` 的转换可用（尽管坐标已在 `base_link` 中，暂未直接使用）。
- **夹爪控制**：使用 `/joint_moveit_ctrl_piper` 控制夹爪（0.035 = 打开，0.0 = 关闭）。
- **到达检查**：依赖关节状态和末端位姿反馈，带有 3 秒超时以确保健壮性。
- **工作空间**：由内半径（0.16 米）和外半径（0.8 米）定义，考虑基座偏移（x：0.091 米，z：0.123 米）。
- **关节限制**：基座关节（joint1）约束在 [-2.62, 2.60] 弧度。

## 设置与执行
1. **依赖项**：
   - ROS 包：`rospkg`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`tf2_ros`、`tf2_geometry_msgs`。
   - MoveIt 服务：`/joint_moveit_ctrl_endpose`、`/joint_moveit_ctrl_piper`。
   - YOLO 检测节点，发布到 `/yolo/m_xyz`。

2. **设置**：
   - 运行 `roscore`、相机节点、YOLO 检测脚本和 MoveIt 服务器。
   - 确保 `camera_link` 到 `base_link` 的 TF 转换可用。
   - 将脚本保存为 `fruit_picker.py` 并赋予可执行权限：
     ```bash
     chmod +x fruit_picker.py
     ```

3. **执行**：
   - 运行脚本：
     ```bash
     rosrun your_package_name fruit_picker.py
     ```
   - 触发循环：
     ```bash
     rostopic pub /wheels/arrive std_msgs/Int32 "data: 1"
     ```

4. **验证**：
   - 监控控制台日志，查看观察、采摘和放置状态。
   - 检查完成信号：
     ```bash
     rostopic echo /yolo/picked
     ```

## 错误处理
- **服务失败**：记录 MoveIt 服务调用失败的错误，跳过失败的动作。
- **无效位姿**：丢弃工作空间外的果实并记录警告。
- **数据缺失**：处理 `/end_pose` 或 `/joint_states` 数据缺失的情况，记录警告并回退到之前的方向。
- **空检测**：推进到下一个观察循环或在无果实时发送完成信号。

## 潜在改进
- **动态果篮位姿**：从参数或话题加载果篮位姿以增加灵活性。
- **方向检查**：在 `check_endpose_arrival` 中添加四元数距离比较，验证末端位姿方向。
- **专用夹爪服务**：若可用，恢复 `/joint_moveit_ctrl_gripper` 用于夹爪控制。
- **阈值调优**：根据机械臂性能调整距离阈值（0.01 米/弧度）或超时（3 秒）。
- **可视化**：集成 RViz 或 OpenCV 可视化检测到的果实和机械臂运动。

## 版本信息
- **最后更新**：2025年7月26日
- **作者**：Xiangyu Sun，由 Grok 3 增强。
- **ROS 版本**：兼容 ROS Noetic 或更高版本（基于 Python 3 使用情况推测）。