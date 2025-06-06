#!/usr/bin/env python3
import rospy
import csv
import pickle
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

def read_csv(filename):
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        times, positions = [], []
        for row in reader:
            times.append(float(row["timestamp"]))
            joint_vals = [float(row[f"joint{i+1}"]) for i in range(6)]
            positions.append(joint_vals)
        return times, positions

def build_trajectory(times, positions, joint_names):
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = joint_names
    for i in range(len(times)):
        point = JointTrajectoryPoint()
        point.positions = positions[i]
        point.time_from_start = rospy.Duration(times[i])
        traj.joint_trajectory.points.append(point)
    return traj

def save_trajectory(traj, filename="recorded_traj.pkl"):
    with open(filename, "wb") as f:
        pickle.dump(traj, f)
    rospy.loginfo(f"Trajectory saved to {filename}")

def main():
    rospy.init_node("build_and_save_traj", anonymous=True)
    times, positions = read_csv("recorded_trajectory.csv")
    joint_names = [f"joint{i+1}" for i in range(6)]  # 或 get from MoveGroupCommander
    traj = build_trajectory(times, positions, joint_names)
    save_trajectory(traj)

if __name__ == "__main__":
    main()
