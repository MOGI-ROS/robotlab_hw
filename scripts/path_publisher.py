#!/usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

def path_publisher():
    rospy.init_node('path_publisher')
    rospy.loginfo("Node initialized.")
    client = actionlib.SimpleActionClient('robot/move_base',MoveBaseAction)
    rospy.loginfo("Connecting to move_base...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base.")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "robot_map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = -0.781250178814
    goal.target_pose.pose.position.y = 1.52264034748
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached.")
    else:
        rospy.loginfo("Failed to reach goal.")
        rospy.loginfo("Status code: " + str(client.get_state()))


if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")