#!/usr/bin/env python
# -*- coding: utf-8 -*-

#MUST BE LAUNCHED INSIDE THE sherpa_ros/scripts folder


from __future__ import division
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import* #PoseStamped PoseArray Pose
import tf
import plantation

n_r = 2
n_c = 2
d_r = 5
d_c = 3
R = 0.6


def getPath(n_r,n_c,d_r,d_c, centers,R):
    points = []
    heading = []
    h=0 
    k=0
    flag = True
    print(centers)
    for c in centers:
	k+=1
        points.append([c[0]+d_c/2,c[1]-d_r/2])
				
        if (h//n_c)%2 == 0:
            heading.append(0)
        else:				
            heading.append(np.pi) 	
	
	last_yaw = heading[-1]

	if k%n_c==0 and flag:
	    points.append([c[0]+d_c,c[1]+R/2])
	    heading.append(np.pi/2)
	    flag =False
	elif k%n_c==0 and not flag:
	    points.append([c[0]-d_c,c[1]+R/2])
	    heading.append(np.pi/2)
	    flag = True		
        h+=1							
        
    l_c = centers[-n_c:]	    
    for c in l_c[::-1]:
        points.append([c[0],c[1]+d_r/2])
        if last_yaw==0:
            heading.append(np.pi)
        else:
            heading.append(0)
    return points,heading

def main():
	centers = plantation.generateCenters(n_r,n_c,d_r,d_c)
	points,heading = getPath(n_r,n_c,d_r,d_c,centers,R)
	#points = [[3,2.5],[6,2.5],[9,2.5],[12,5],[9,7.5],[6,7.5]]
	#heading=[0,0,0,np.pi/2,np.pi,np.pi]
	print(points)
	print(heading)
	rospy.init_node('global_path', anonymous=True)
	rate =rospy.Rate(10)
	pub = rospy.Publisher('/path_sherpa', Path, queue_size=10)
	pub2 = rospy.Publisher('/target_trees', PoseArray, queue_size=10)
	msg_path = Path()
	msg_target_trees = PoseArray();

	for i in range(len(heading)):
		msg_pose = PoseStamped()
		p = points[i]
		msg_pose.pose.position.x = p[0]
		msg_pose.pose.position.y = p[1]
		quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,heading[i])
		msg_pose.pose.orientation.x = quaternion[0]
		msg_pose.pose.orientation.y = quaternion[1]
		msg_pose.pose.orientation.z = quaternion[2]
		msg_pose.pose.orientation.w = quaternion[3]
		msg_path.poses.append(msg_pose)
	
	for j in range(len(centers)):
	    msg_trees_pose = Pose()
	    c = centers[j]
	    msg_trees_pose.position.x = c[0]
	    msg_trees_pose.position.y = c[1]
	    msg_trees_pose.position.z = 0.0
	    msg_trees_pose.orientation.x = 0.0
	    msg_trees_pose.orientation.y = 0.0
	    msg_trees_pose.orientation.z = 0.0
	    msg_trees_pose.orientation.w = 0.0
	    msg_target_trees.poses.append(msg_trees_pose)	

	while not rospy.is_shutdown():
		
		pub.publish(msg_path)	
		pub2.publish(msg_target_trees)		
		rate.sleep()		
	
	
  
if __name__ == '__main__':  
    main()
