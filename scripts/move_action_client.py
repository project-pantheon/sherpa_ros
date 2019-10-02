#! /usr/bin/env python
import rospy
import actionlib
import std_msgs
import local_planner

from sherpa_ros.msg import MoveAction, MoveGoal, MoveResult
from nav_msgs.msg import Odometry



start = None
end = None

class sub_to_odom:
    def  __init__(self,x,y,yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
        
def goal_pose(pose):
	goal = MoveGoal()
	#odom = Odometry()
	goal.goal.pose.pose.position.x = pose[0]
	goal.goal.pose.pose.position.y = pose[1]
	goal.goal.pose.pose.orientation.z = pose[2]
	#goal.goal = odom
	
	return goal

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = msg.pose.pose.orientation.z
    global end 
    end = sub_to_odom(x, y, yaw) 
    
def callback2(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = msg.pose.pose.orientation.z
    global start 
    start = sub_to_odom(x, y, yaw) 
    
    

if __name__=='__main__':

	rospy.init_node('move_action_client')
	client = actionlib.SimpleActionClient('move',MoveAction)
	client.wait_for_server()
	
	while end is None:
	    sub_to_goal = rospy.Subscriber('sherpa/goal', Odometry, callback)
	    sub_to_gt = rospy.Subscriber('sherpa/ground_truth', Odometry, callback2)
	
	waypoints = local_planner.main(start.x, start.y, start.yaw, end.x, end.y, end.yaw)

	
	
	for way in waypoints:						
		goal = goal_pose(way)
		client.send_goal(goal)
		client.wait_for_result()
		print(way)
		print 'Time elapsed: %f'%(client.get_result().time_elapsed.to_sec())
	
		
