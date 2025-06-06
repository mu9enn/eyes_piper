# import numpy as np
# import matplotlib.pyplot as plt
#
# # 连杆长度
# L1 = 285.05  # mm
# L2 = 250.65  # mm
# L3 = 91.0  # mm
#
# # 角度限制 (单位：度)
# theta1_range = np.radians([0, 180.0])  # 转换为弧度
# theta2_range = np.radians([-170, 0])  # 转换为弧度
# theta3_range = np.radians([-100.0, 100.0])  # 转换为弧度
#
#
# # 计算末端位置的函数
# def forward_kinematics(theta1, theta2, theta3):
#     # 角度关系
#     theta3 = -theta1 - theta2
#
#     # 使用正向运动学计算末端执行器位置
#     x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
#     y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
#
#     return x, y
#
#
# # 采样角度
# theta1_samples = np.linspace(theta1_range[0], theta1_range[1], 150)
# theta2_samples = np.linspace(theta2_range[0], theta2_range[1], 150)
#
# # 创建空列表用于存储末端执行器的位置
# end_effector_positions = []
#
# # 进行采样并计算末端执行器的位置
# for theta1 in theta1_samples:
#     for theta2 in theta2_samples:
#         # 计算theta3
#         theta3 = -theta1 - theta2
#         # 计算末端执行器的位置
#         x, y = forward_kinematics(theta1, theta2, theta3)
#         end_effector_positions.append([x, y])
#
# # 转换成numpy数组，便于绘图
# end_effector_positions = np.array(end_effector_positions)
#
# # 绘制工作空间
# plt.figure(figsize=(20, 20))  # 图像大小设置为20x20
# plt.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], s=1, c='blue', marker='.')
#
# # 设置图表标题与标签
# plt.title("Workspace of the 3-Link Planar Manipulator")
# plt.xlabel("X (mm)")
# plt.ylabel("Y (mm)")
#
# # 设置轴范围
# plt.xlim(-1000, 1000)
# plt.ylim(-1000, 1000)
#
# # 设置刻度
# plt.xticks(np.arange(-1000, 1001, 10))  # 每10mm一条刻度线
# plt.yticks(np.arange(-1000, 1001, 10))  # 每10mm一条刻度线
#
# # 设置网格
# plt.grid(True)
#
# # 设置图表比例一致
# plt.axis('equal')
#
# # 加粗0刻度线
# plt.axhline(y=0, color='black', linewidth=2)  # 水平0刻度线
# plt.axvline(x=0, color='black', linewidth=2)  # 垂直0刻度线
#
# # 显示图像
# plt.show()

import numpy as np
import matplotlib.pyplot as plt

# 连杆长度
L1 = 285.05  # mm
L2 = 250.65  # mm
L3 = 91.0  # mm

# 给定的关节角度（单位：度，转换为弧度）
theta1 = np.radians(45)  # 示例角度
theta2 = np.radians(-90)  # 示例角度
theta3 = np.radians(45)  # 示例角度


# 计算每个连杆的端点位置
def calculate_link_positions(theta1, theta2, theta3):
    # 计算第一根连杆的端点
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    # 计算第二根连杆的端点
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    # 计算第三根连杆的端点
    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + L3 * np.sin(theta1 + theta2 + theta3)

    return (x1, y1), (x2, y2), (x3, y3)


# 获取连杆端点
link1_end, link2_end, link3_end = calculate_link_positions(theta1, theta2, theta3)

# 绘制三个连杆
plt.figure(figsize=(8, 8))
plt.plot([0, link1_end[0]], [0, link1_end[1]], label="Link 1", color='r', linewidth=4)
plt.plot([link1_end[0], link2_end[0]], [link1_end[1], link2_end[1]], label="Link 2", color='g', linewidth=4)
plt.plot([link2_end[0], link3_end[0]], [link2_end[1], link3_end[1]], label="Link 3", color='b', linewidth=4)

# 标记各连杆端点
plt.scatter([link1_end[0], link2_end[0], link3_end[0]], [link1_end[1], link2_end[1], link3_end[1]], color='black')

# 设置标题和标签
plt.title("3-Link Planar Manipulator")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.axis("equal")
plt.grid(True)

# 显示图例
plt.legend()
plt.show()
