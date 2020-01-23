#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
import csv
import math
import numpy


global pub, path, enable_global_planner, prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_coordinates, new_waypoint

path='/home/renzo/lab_ws/src/sherpa_ros/scripts/'

enable_global_planner = False
new_waypoint = True
waypoint_id = 0
threshold=0.1

def updateWaypointFromTour():

    global prev_waypoint, waypoint, waypoint_id, map_coordinates, waypoint_coordinates, new_waypoint

    if new_waypoint :
        i=0
        for elem in map_coordinates:
            if elem[2] == waypoint_coordinates[waypoint_id]:
                waypoint.x=map_coordinates[i][0]
                waypoint.y=map_coordinates[i][1]
                if waypoint_id>0 :
                    waypoint.z=math.atan2(waypoint.y-prev_waypoint.y, waypoint.x-prev_waypoint.x)
                else :
                    waypoint.z=0
                new_waypoint = False
                break
            i=i+1
    

def runGlobalPlannerService(call):

    global enable_global_planner

    enable_global_planner = True
    print "global_planner service called"

    return EmptyResponse()


def odomCallback(odomData):

    global prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_coordinates, enable_global_planner, new_waypoint

    # distance
    a = numpy.array((odomData.pose.pose.position.x, odomData.pose.pose.position.y))
    b = numpy.array((waypoint.x, waypoint.y))
    dist = numpy.linalg.norm(a-b)

    # check distance
    if waypoint_id+1<max_waypoint_id and enable_global_planner :
        if dist<=threshold :
            if waypoint_id>0 :
                prev_waypoint.x = waypoint.x
                prev_waypoint.y = waypoint.y
                prev_waypoint.z = waypoint.z
            waypoint_id=waypoint_id+1
            new_waypoint = True

    updateWaypointFromTour()

    print "-----"
    print "Waypoint_ID: ", waypoint_id+1
    print "Waypoint: \n", waypoint
    print "Position: \n", odomData.pose.pose.position
    

def global_planner():

    global pub, path, enable_global_planner, prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_coordinates

    #map
    with open(path+'virtual_map_points.csv', 'rb') as csvfile:
        mapData = list(csv.reader(csvfile))

    #cast map to float
    map_coordinates = [[0 for x in range(3)] for y in range(len(mapData))]
    i=0
    for elem in mapData:
        map_coordinates[i][0] = float(elem[0])
        map_coordinates[i][1] = float(elem[1])
        map_coordinates[i][2] = float(elem[2])
        i=i+1

    #tour
    with open(path+'tour.csv', 'rb') as csvfile:
        tourData = list(csv.reader(csvfile))

    #cast tour to float
    waypoint_coordinates = [[0 for x in range(1)] for y in range(len(tourData))]
    i=0
    for elem in tourData:
        waypoint_coordinates[i] = float(elem[0])
        i=i+1

    #set last waypoint
    max_waypoint_id= len(tourData)

    #ROS init
    rospy.init_node('global_planner', anonymous=True)
    waypoint=Point()
    prev_waypoint=Point()
    pub = rospy.Publisher('/waypoint', Point, queue_size=1)
    rospy.Subscriber("/odom_gps", Odometry, odomCallback)
    s_print = rospy.Service('run_global_planner', Empty, runGlobalPlannerService)

    #main loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if enable_global_planner :
            pub.publish(waypoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        global_planner()
    except rospy.ROSInterruptException:
        pass

