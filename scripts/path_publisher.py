#!/usr/bin/env python
import rospy
import tf
import actionlib
import numpy

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

pi = 3.14159

def path_publisher():
    rospy.init_node('path_publisher')
    rospy.loginfo("Node initialized.")
    base_point = numpy.array([[-7.5, 3.1, pi]])
    table_1 = numpy.array([[-5.5, 2.1, pi/2]])
    table_2 = numpy.array([[-3.75, 2.1, pi/2]])
    table_3 = numpy.array([[-2.8, 2.1, pi/2]])
    table_4 = numpy.array([[-1.1, 2.1, pi/2]])
    table_5 = numpy.array([[-5.1, 0.9, 3*pi/2]])
    table_6 = numpy.array([[-3.1, 0.9, 3*pi/2]])
    table_7 = numpy.array([[-1.45, 0.9, 3*pi/2]])
    table_8 = numpy.array([[0.5, 0.9, 3*pi/2]])
    goals = numpy.vstack(
        (base_point,
        table_1,
        table_2,
        table_6,
        table_8,
        base_point)
    )

    client = actionlib.SimpleActionClient('robot/move_base',MoveBaseAction)
    rospy.loginfo("Connecting to move_base...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base.")
    loop_path = True
    while loop_path:
        loop_path = rospy.get_param('~loop_path', False)
        for i in range(0, goals.shape[0]):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "robot_map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = goals[i][0]
            goal.target_pose.pose.position.y = goals[i][1]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, goals[i][2]))

            rospy.loginfo("Sending goal...")
            client.send_goal(goal)
            client.wait_for_result()
        
            if client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached.")
            else:
                rospy.loginfo("Failed to reach goal.")
                rospy.loginfo("Status code: " + str(client.get_state()))
            rospy.loginfo("Waiting before sending next goal...")
            rospy.sleep(3)
        rospy.loginfo("Reached end of goals.")
        if loop_path:
            rospy.loginfo("Looping is set to true, restarting from first goal...")
        else:
            rospy.loginfo("Looping is set to false, exiting...")
            break


if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")