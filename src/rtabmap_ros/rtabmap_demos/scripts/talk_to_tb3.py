#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Predefined goal positions
GOAL_LOCATIONS = {
    "test": {"x": -5.0, "y": 0.5, "yaw": -1.57},
    "stairs": {"x": 20.0, "y": 7.0, "yaw": 0.0},
    "truck port": {"x": -7.0, "y": 0.0, "yaw": 3.13}
}

def send_goal(location_name, coords):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = coords["x"]
    goal.target_pose.pose.position.y = coords["y"]
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.w = 1.0  # Simplified, facing forward

    rospy.loginfo(f"Sending robot to {location_name}...")
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"✅ Arrived at {location_name}")
    else:
        rospy.logwarn(f"⚠️ Failed to reach {location_name}")

if __name__ == '__main__':
    try:
        rospy.init_node('interactive_goal_sender')

        while not rospy.is_shutdown():
            user_input = input("Where should the robot go? (e.g. 'Travel to stairs'):\n").lower()

            if "stairs" in user_input:
                send_goal("stairs", GOAL_LOCATIONS["stairs"])

            elif "truck" in user_input or "port" in user_input:
                send_goal("truck port", GOAL_LOCATIONS["truck port"])

            elif user_input in ["exit", "quit"]:
                print("Exiting.")
                break

            else:
                print("Sorry, I don't know that location.")

    except rospy.ROSInterruptException:
        pass
