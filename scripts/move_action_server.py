#! /usr/bin/env python
import rospy
import time
import actionlib
import std_msgs
from sherpa_ros.msg import MoveAction, MoveGoal, MoveResult
from nav_msgs.msg import Odometry

error = 0.02
varS = None


def callback(msg):
	global varS
	varS = msg.pose.pose.position

def check_position(pose,way):
	
	if (way.x-error < pose.x and pose.x < way.x+error) and (way.y-error < pose.y and pose.y < way.y+error):
		return True
	return False

def do_move(goal):
	start_time = time.time()
	way = goal.goal.pose.pose.position
	pub.publish(goal.goal)
	while not check_position(varS,way):
		pass
	result = MoveResult()
	result.time_elapsed = rospy.Duration.from_sec(time.time()-start_time)
	server.set_succeeded(result)
	
rospy.init_node('move_action_server')
pub=rospy.Publisher('sherpa/way_point',Odometry,queue_size=10)
sub = rospy.Subscriber('sherpa/ground_truth', Odometry, callback)
server = actionlib.SimpleActionServer('move',MoveAction, do_move,False)
server.start()
rospy.spin()
