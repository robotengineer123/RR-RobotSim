#! /usr/bin/env python
import rospy
import sys
import moveit_commander
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionGoal
import actionlib

def SendGoal(client: actionlib.SimpleActionClient, x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1
    
    client.send_goal(goal)
    client.wait_for_result()

def main():
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    

    SendGoal(client, 6, 0.5)
    rospy.sleep(1)
    SendGoal(client, 0, -0.5)

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.sleep(50)
    rospy.init_node('goal_sender')
    main()
