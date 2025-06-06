#!/usr/bin/env python3
import rospy
import pickle
import moveit_commander

def load_trajectory(filename="recorded_traj.pkl"):
    with open(filename, "rb") as f:
        traj = pickle.load(f)
    return traj

def execute_trajectory(traj):
    moveit_commander.roscpp_initialize([])
    rospy.init_node("play_saved_traj", anonymous=True)

    group = moveit_commander.MoveGroupCommander("arm")  # 可改为 "piper"
    rospy.loginfo("Executing pre-built trajectory...")
    group.execute(traj, wait=True)
    moveit_commander.roscpp_shutdown()

def main():
    traj = load_trajectory()
    execute_trajectory(traj)

if __name__ == "__main__":
    main()
