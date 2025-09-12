#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import csv

recorded_data = []

def callback(msg):
    now = rospy.Time.now().to_sec()
    # 前6个关节为arm（joint1~joint6）
    joints = list(msg.position[:6])
    recorded_data.append([now] + joints)

def main():
    rospy.init_node("record_trajectory")
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.loginfo("Recording... Ctrl+C to stop")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        with open("recorded_trajectory.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp"] + [f"joint{i+1}" for i in range(6)])
            start_time = recorded_data[0][0]
            for row in recorded_data:
                # 用相对时间戳（便于后续控制）
                rel_time = row[0] - start_time
                writer.writerow([rel_time] + row[1:])
        rospy.loginfo("Saved to recorded_trajectory.csv")

if __name__ == "__main__":
    main()
