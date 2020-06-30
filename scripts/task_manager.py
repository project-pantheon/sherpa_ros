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

from os.path import expanduser

global pub, path, enable_task_manager, prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, new_waypoint, tour_filename, origin_x, origin_y, origin_Y, cnt

path = expanduser("~") + '/catkin_ws/src/sherpa_ros/scripts/'
tour_filename='tour'
#origin=final position after last waypoint
origin_x=0
origin_y=0
origin_Y=0
origin_orientation= False

enable_task_manager = False
new_waypoint = True
waypoint_id = 0
threshold=0.3

gimbal_values=[0]

tmp=1

def updateWaypointFromTour():

    global prev_waypoint, waypoint, waypoint_id, map_coordinates, waypoint_data, new_waypoint

    if new_waypoint :
        i=0
        for elem in map_coordinates:
            if elem[2] == waypoint_data[waypoint_id][0]:
                waypoint.x=map_coordinates[i][0]
                waypoint.y=map_coordinates[i][1]
                waypoint.z=waypoint_data[waypoint_id][1] 

#                if waypoint_id>0 :
#                    waypoint.z=math.atan2(waypoint.y-prev_waypoint.y, waypoint.x-prev_waypoint.x)
#                else :
#                    waypoint.z=0


                new_waypoint = False
                break
            i=i+1

def check_go2origin(a):
    
    global prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, enable_task_manager, new_waypoint, tour_filename, origin_orientation, origin_x, origin_y, origin_Y

    if waypoint_id+1==max_waypoint_id and enable_task_manager :
        # update waypoint and dist
        waypoint.x=origin_x
        waypoint.y=origin_y

        if origin_orientation :
            waypoint.z=origin_Y
        else :
            waypoint.z=math.atan2(waypoint.y-prev_waypoint.y, waypoint.x-prev_waypoint.x)
        
        b = numpy.array((waypoint.x, waypoint.y))
        dist = numpy.linalg.norm(a-b)

        print "-----"
        print "dist: ", dist
        print "real_waypoint_ID: ", waypoint_id
        print "x=", origin_x, ", y=", origin_y
        print "Waypoint: \n", waypoint
        print "Position: \n", a

        if dist<=threshold :
            enable_task_manager = False
            print "task_manager end: dist=", dist


def runTaskManagerService(call):

    global enable_task_manager

    enable_task_manager = True
    print "task_manager service called"

    return EmptyResponse()


def odomCallback(odomData):

    global prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, enable_task_manager, new_waypoint, tour_filename, tmp

    # distance
    a = numpy.array((odomData.pose.pose.position.x, odomData.pose.pose.position.y))
    b = numpy.array((waypoint.x, waypoint.y))
    dist = numpy.linalg.norm(a-b)

    # check distance
    if waypoint_id+1<max_waypoint_id and enable_task_manager :
        if dist<=threshold :

            # check if scanning action is required
            if waypoint_data[waypoint_id][2] >= 1:

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


                #time.sleep(waypoint_data[waypoint_id][1]*45)

                #set scanning done
                waypoint_data[waypoint_id][2]=0
            else:
                if waypoint_id>0 :
                    prev_waypoint.x = waypoint.x
                    prev_waypoint.y = waypoint.y
                    prev_waypoint.z = waypoint.z
                waypoint_id=waypoint_id+1
                new_waypoint = True

    updateWaypointFromTour()
    check_go2origin(a)

    print "-----"
    print "tour: ", tour_filename
    print "Waypoint_ID: ", waypoint_id+1
    print "Waypoint: \n", waypoint
    print "Position: \n", odomData.pose.pose.position
    

def task_manager():

    global pub, path, enable_task_manager, prev_waypoint, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, tour_filename, origin_x, origin_y, origin_Y

    #ROS init
    rospy.init_node('task_manager', anonymous=True)

    if rospy.has_param('~tour'):
        tour_filename=rospy.get_param('~tour')

    if rospy.has_param('~origin_x'):
        origin_x=rospy.get_param('~origin_x')

    if rospy.has_param('~origin_y'):
        origin_y=rospy.get_param('~origin_y')

    if rospy.has_param('~origin_Y'):
        origin_orientation=True
        origin_Y=rospy.get_param('~origin_Y')

    if rospy.has_param('~odometry_topic'):
        odometry_topic=rospy.get_param('~odometry_topic')
    else :
        odometry_topic=('/odom_gps')

    if rospy.has_param('~map_points'):
        map_points=rospy.get_param('~map_points')
    else :
        map_points=('virtual_map_points')

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

    waypoint=Point()
    prev_waypoint=Point()
    #prev_waypoint.x=origin_x
    #prev_waypoint.y=origin_y

    pub = rospy.Publisher('/waypoint', Point, queue_size=1)
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

