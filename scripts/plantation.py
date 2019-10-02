#!/usr/bin/env python
from __future__ import division
import sys, rospy, tf
import numpy as np
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
import random as rand
import subprocess



#default 5 - 8
def generateCenters(num_rows = 3, num_col=3, dist_row = 5, dist_col=3):
     
    xx=0
    yy =0
    centers = []
    h =0;
    for i in range(0,num_rows):
  	yy +=dist_row
        xx=0
	l_c=[]
        for j in range(0,num_col):
            xx+=dist_col  
            l_c.append([xx,yy])
            h+=1
	if i%2 ==0:
            centers.extend(l_c)
        else:
	    centers.extend(l_c[::-1])
        
    return centers 

def main():
	
	rospy.init_node("spawn_plantation")
	rospy.wait_for_service("gazebo/delete_model")
	rospy.wait_for_service("gazebo/spawn_sdf_model")
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        delete_model("plantation")
	s = rospy.ServiceProxy("gazebo/spawn_sdf_model",SpawnModel)
	#orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

	
	#with open("/home/pasquale/catkin_ws/src/sherpa_ros/scripts/cylinder.sdf","r") as f:
	with open("/home/ciro/.gazebo/models/tree/model.sdf","r") as f:
		tree_xml =f.read()
	trees = generateCenters()
	print(trees)
	i = 0
	for t in trees:
		tree_name = "unit_cylinder_%d" % (i)
		delete_model(tree_name)
		orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rand.uniform(0,6.24)))
		pose = Pose(Point(t[0],t[1],0.0),orient)
		s(tree_name, tree_xml,"",pose,"world")
		i+=1
		

if __name__=='__main__':
	main()
