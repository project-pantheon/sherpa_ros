#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry  
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
	
def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map' 
	goal_pose.target_pose.pose.position.x = pose.pose.position.x
	goal_pose.target_pose.pose.position.y = pose.pose.position.y
	goal_pose.target_pose.pose.position.z = pose.pose.position.z
	goal_pose.target_pose.pose.orientation.x = pose.pose.orientation.x
	goal_pose.target_pose.pose.orientation.y = pose.pose.orientation.y
	goal_pose.target_pose.pose.orientation.z = pose.pose.orientation.z
	goal_pose.target_pose.pose.orientation.w = pose.pose.orientation.w
	return goal_pose

def callback_path(msg):
	global waypoints
	waypoints = msg.poses	

def callback_odom(msg):
	global x
	global y
	x = msg.pose.position.x
	y = msg.pose.position.y
	

if __name__ == '__main__':

	waypoints = []
	rospy.init_node('patrol')
	rate = rospy.Rate(10)
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	sub = rospy.Subscriber('/path_sherpa' , Path, callback_path)
	rate.sleep()
	
	while True:
		for pose in waypoints:
			
			goal = goal_pose(pose)
			client.send_goal(goal)
			client.wait_for_result()

