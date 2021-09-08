#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from pyproj import Proj
import json
import math
import csv
import tf

global path

path='/home/sherpa/catkin_ws/src/sherpa_ros/scripts/suckers/'

def path_csv2ros():

    global path, map_points,mapData

    rospy.init_node('check_point', anonymous=True)

    #map
    with open(path+ 'sucker_newline_32587_7841781773703387716.txt', 'rb') as csvfile:
        mapData = list(csv.reader(csvfile))	

    print mapData

    map_coordinates = [[0 for x in range(3)] for y in range(len(mapData))]

    i=0
    for elem in mapData:
        map_coordinates[i][0] = float(elem[4])
        map_coordinates[i][1] = float(elem[5])
        map_coordinates[i][2] = 0
        i=i+1

    tf_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        i=0
        for elem in map_coordinates:
            
            target_str="target_"+str(i)
            tf_broadcaster.sendTransform(
                (elem[0], elem[1], elem[2]),
                (0, 0, 0, 1),
                rospy.Time.now(),
                target_str, "map")
            i=i+1
        rate.sleep()
        

if __name__ == '__main__':
    try:
        path_csv2ros()
    except rospy.ROSInterruptException:
        pass
