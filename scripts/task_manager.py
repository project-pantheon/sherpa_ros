#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from scanning_controller.srv import ScanningTree, ScanningTreeResponse, ScanningTreeRequest, OffsetSet, OffsetSetResponse, OffsetSetRequest
import csv
import math
import numpy
import time
import tf
import os
from os.path import expanduser


global pub, path, enable_task_manager, enable_scanning, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, new_waypoint, tour_filename, cnt

default_workspace_value=expanduser("~")+'/catkin_ws'
path = os.getenv('ROS_WORKSPACE', default_workspace_value)+'/src/sherpa_ros/scripts/'

tour_filename='tour_warehouse'

enable_task_manager = False
enable_scanning = True
new_waypoint = True
waypoint_id = 0
threshold=0.3

gimbal_values=[0]

tmp=1

def updateWaypointFromTour():

    global waypoint, waypoint_id, map_coordinates, waypoint_data, new_waypoint

    if new_waypoint :
        i=0
        for elem in map_coordinates:
            if elem[2] == waypoint_data[waypoint_id][0]:
                waypoint.pose.pose.position.x=map_coordinates[i][0]
                waypoint.pose.pose.position.y=map_coordinates[i][1]

                quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint_data[waypoint_id][1])
                waypoint.pose.pose.orientation.x = quaternion[0]
                waypoint.pose.pose.orientation.y = quaternion[1]
                waypoint.pose.pose.orientation.z = quaternion[2]
                waypoint.pose.pose.orientation.w = quaternion[3]

                new_waypoint = False
                break
            i=i+1


def runTaskManagerService(call):

    global enable_task_manager

    enable_task_manager = True
    print "task_manager service called"

    return EmptyResponse()


def odomCallback(odomData):

    global waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, enable_task_manager, enable_scanning, new_waypoint, tour_filename, tmp

    # distance
    a = numpy.array((odomData.pose.pose.position.x, odomData.pose.pose.position.y))
    b = numpy.array((waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
    dist = numpy.linalg.norm(a-b)

    # check distance
    if waypoint_id+1<max_waypoint_id and enable_task_manager :
        if dist<=threshold :

            # check if scanning action is required
            if waypoint_data[waypoint_id][2] >= 1 :

                if enable_scanning :

                    #lock scanning operations

                    #avvia richiesta action

                    #set scanning end effector position
                    rospy.wait_for_service('/offset_set')
                    try:
                        offset_set_client = rospy.ServiceProxy('/offset_set', OffsetSet)

                        #calculate offsets
                        offset1=-2.0
                        offset2=-0.9

                        resp = offset_set_client(offset1, offset2)
                    except rospy.ServiceException, e:
                        print "Task Manager: OffsetSet service call failed: %s"%e

                    #scanning calls
                    rospy.wait_for_service('/scan_tree')
                    try:
                        scan_tree_client = rospy.ServiceProxy('/scan_tree', ScanningTree)

                        #calculate (1 front/ 2 rear/ 3 front+rear)
    #                    scan_type=1

    #                    #loop for can only one time one tree
    #                    scan_type=tmp
    #                    if tmp == 1:
    #                        tmp=2
    #                    else:
    #                        tmp=1
                        
                        #global planner codification relative to the robot [45 135 -135 -45] deg
                        if waypoint_data[waypoint_id][6]==1 & waypoint_data[waypoint_id][5] ==1 :
                            scan_type =3
                        elif waypoint_data[waypoint_id][6]==1 :
                            scan_type =1
                        elif waypoint_data[waypoint_id][5]==1 :
                            scan_type =2

                        resp = scan_tree_client(scan_type)
                    except rospy.ServiceException, e:
                        print "Task Manager: ScanningTree service call failed: %s"%e

                else :
                    time.sleep(waypoint_data[waypoint_id][2]*10)

                #set scanning done
                waypoint_data[waypoint_id][2]=0
            else:
                waypoint_id=waypoint_id+1
                new_waypoint = True

    updateWaypointFromTour()

    print "-----"
    print "tour: ", tour_filename
    print "Waypoint_ID: ", waypoint_id+1
    print "Waypoint_position: \n", waypoint.pose.pose.position
    print "Position_position: \n", odomData.pose.pose.position

    quaternion = (
        waypoint.pose.pose.orientation.x,
        waypoint.pose.pose.orientation.y,
        waypoint.pose.pose.orientation.z,
        waypoint.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print "Waypoint_orientation: \n", euler[2]
    quaternion = (
        odomData.pose.pose.orientation.x,
        odomData.pose.pose.orientation.y,
        odomData.pose.pose.orientation.z,
        odomData.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print "Position_orientation: \n", euler[2]

def task_manager():

    global pub, path, enable_task_manager, enable_scanning, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, tour_filename

    #ROS init
    rospy.init_node('task_manager', anonymous=True)

    if rospy.has_param('~tour'):
        tour_filename=rospy.get_param('~tour')

    if rospy.has_param('~odometry_topic'):
        odometry_topic=rospy.get_param('~odometry_topic')
    else :
        odometry_topic=('/odom_gps')

    if rospy.has_param('~enable_scanning'):
        enable_scanning=rospy.get_param('~enable_scanning')
    else :
        enable_scanning=(True)

    if rospy.has_param('~map_points'):
        map_points=rospy.get_param('~map_points')
    else :
        map_points=('virtual_map_points')


    enable_scanning

    #map
    with open(path+ map_points+'.csv', 'rb') as csvfile:
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
    with open(path+tour_filename+'.csv', 'rb') as csvfile:
        tourData = list(csv.reader(csvfile))

    #cast tour to float
    waypoint_data = [[0 for x in range(7)] for y in range(len(tourData))]
    i=0
    for elem in tourData:
        waypoint_data[i][0] = float(elem[0])
        waypoint_data[i][1] = float(elem[1])
        waypoint_data[i][2] = int(elem[2])
        waypoint_data[i][3] = int(elem[3])
        waypoint_data[i][4] = int(elem[4])
        waypoint_data[i][5] = int(elem[5])
        waypoint_data[i][6] = int(elem[6])
        i=i+1

    print waypoint_data

    #set last waypoint
    max_waypoint_id= len(tourData)

    waypoint=Odometry()

    pub = rospy.Publisher('/command/pose', Odometry, queue_size=1)
    rospy.Subscriber(odometry_topic, Odometry, odomCallback)
    s_glob = rospy.Service('run_task_manager', Empty, runTaskManagerService)

    #main loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if enable_task_manager :
            pub.publish(waypoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        task_manager()
    except rospy.ROSInterruptException:
        pass

