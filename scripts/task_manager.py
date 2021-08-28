#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from scanning_controller.srv import ScanningTree, ScanningTreeResponse, ScanningTreeRequest, OffsetSet, OffsetSetResponse, OffsetSetRequest
from rm3_ackermann_controller.srv import ActivateController, ActivateControllerResponse, ActivateControllerRequest
from pyproj import Proj
from spray_controller.srv import AimingSpraying, AimingSprayingResponse

import csv
import math
import numpy
import time
import tf
import os
from os.path import expanduser

global pub, path, enable_task_manager, enable_task, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, new_waypoint, tour_filename, cnt, threshold_waypoint, task_name, origin_utm_lon, origin_utm_lat, origin_lon, origin_lat, origin_set

default_workspace_value=expanduser("~")+'/catkin_ws'
path = os.getenv('ROS_WORKSPACE', default_workspace_value)+'/src/sherpa_ros/scripts/'

tour_filename='tour_warehouse'

enable_task_manager = False
enable_task = True
new_waypoint = True
waypoint_id = 0
threshold=0.2
threshold_waypoint=0.6

#origin_lat=42.2800659
#origin_lon=12.3013824
origin_utm_lat=0
origin_utm_lon=0
origin_set=False

gimbal_values=[0]

tmp=1

def runTask():
    global task_name
    if task_name == 'scanning':
        scanningTask()
    elif task_name == 'spraying':
        sprayingTask()
    else :
        print('task not defined')


def scanningTask():
    #set scanning end effector position
    rospy.wait_for_service('/offset_set')
    try:
        offset_set_client = rospy.ServiceProxy('/offset_set', OffsetSet)

        #calculate offsets
        offset1=2.2
        offset2=1.1
        offset3=-1.1
        offset4=-2.2

        resp = offset_set_client(offset1,offset2,offset3,offset4)
    except rospy.ServiceException, e:
        print "Task Manager: OffsetSet service call failed: %s"%e

    #scanning calls
    rospy.wait_for_service('/scan_tree')
    try:
        scan_tree_client = rospy.ServiceProxy('/scan_tree', ScanningTree)
        scan_type = [waypoint_data[waypoint_id][3], waypoint_data[waypoint_id][4], waypoint_data[waypoint_id][5], waypoint_data[waypoint_id][6] ]
        resp = scan_tree_client(scan_type)
    except rospy.ServiceException, e:
        print "Task Manager: ScanningTree service call failed: %s"%e

def sprayingTask():

    global waypoint_data, waypoint_id

    #activation manager_suckers
    if (waypoint_data[waypoint_id][2]==1):
        rospy.wait_for_service('/manager_suckers/activate_nodes')
        print "Requested activation for manager suckers"
        try:
            manager_suckers_activation_client = rospy.ServiceProxy('/manager_suckers/activate_nodes', Empty)
            resp = manager_suckers_activation_client()
        except rospy.ServiceException, e:
            print "Task Manager: manager_suckers service call failed: %s"%e    
    elif (waypoint_data[waypoint_id][2]==2):
        # Call service for computing suckers
        rospy.wait_for_service('/manager_suckers/compute_mesh_area')
        try:
            manager_suckers_request_client = rospy.ServiceProxy('/manager_suckers/compute_mesh_area', Empty)
            resp = manager_suckers_request_client()
        except rospy.ServiceException, e:
            print "Task Manager: manager_suckers service call failed: %s"%e    
        # Call service for parsing
        rospy.wait_for_service('/landmark_parser/parsing')
        try:
            landmark_parsing_client = rospy.ServiceProxy('/landmark_parser/parsing', Empty)
            resp = landmark_parsing_client()
        except rospy.ServiceException, e:
            print "Task Manager: LandmarkParser Parsing service call failed: %s"%e
        # Call service for tour creation
        rospy.wait_for_service('/landmark_parser/tour')
        try:
            landmark_tour_client = rospy.ServiceProxy('/landmark_parser/tour', Empty)
            resp = landmark_tour_client()
        except rospy.ServiceException, e:
            print "Task Manager: LandmarkParser Tour service call failed: %s"%e

    elif (waypoint_data[waypoint_id][2]==3):
        #required publication of target

        # Call service for aiming and spraying
        rospy.wait_for_service('/auto_aim_spray_fire')
        try:
            manager_suckers_client = rospy.ServiceProxy('/auto_aim_spray_fire', AimingSpraying)
            target=Point(waypoint_data[waypoint_id][3],waypoint_data[waypoint_id][4],waypoint_data[waypoint_id][5])
            resp = manager_suckers_client(waypoint_data[waypoint_id][6],target)
        except rospy.ServiceException, e:
            print "Task Manager: manager_suckers service call failed: %s"%e


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

def robotMovementsEnable(status):
    rospy.wait_for_service('/activate_controller')
    try:
        activate_controller_client = rospy.ServiceProxy('/activate_controller', ActivateController)

        resp = activate_controller_client(status)
    except rospy.ServiceException, e:
        print "Task Manager: Activate_controller service call failed: %s"%e


def odomCallback(odomData):

    global waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, enable_task_manager, enable_task, new_waypoint, tour_filename, tmp, threshold_waypoint, task_name

    # distance
    a = numpy.array((odomData.pose.pose.position.x, odomData.pose.pose.position.y))
    b = numpy.array((waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
    dist = numpy.linalg.norm(a-b)
    
    # check distance
    if waypoint_id+1<=max_waypoint_id and enable_task_manager :
        skip_step=False
        if dist<threshold_waypoint :
            if waypoint_data[waypoint_id][2] == 0:
                skip_step=True
                waypoint_id=waypoint_id+1
                new_waypoint = True

        if not skip_step and dist<threshold :
            #time.sleep(5)
            
            # check if scanning action is required
            if waypoint_data[waypoint_id][2] >= 1 :

                if enable_task :

                    #robot movements disable
                    robotMovementsEnable(False)

                    #run Task
                    runTask()

                    #robot movements enable
                    robotMovementsEnable(True)
                    
                else :
                    time.sleep(waypoint_data[waypoint_id][2]*5)

                #set scanning done
                waypoint_data[waypoint_id][2]=0
            else:
                waypoint_id=waypoint_id+1
                new_waypoint = True
    
    if waypoint_id+1<=max_waypoint_id:
        updateWaypointFromTour()

        print "-----"
        print "tour: ", tour_filename
        print "Waypoint_ID: ", waypoint_id+1
#        print "Waypoint_position: \n", waypoint.pose.pose.position
#        print "Position_position: \n", odomData.pose.pose.position

        quaternion = (
            waypoint.pose.pose.orientation.x,
            waypoint.pose.pose.orientation.y,
            waypoint.pose.pose.orientation.z,
            waypoint.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
#        print "Waypoint_orientation: \n", euler[2]
        quaternion = (
            odomData.pose.pose.orientation.x,
            odomData.pose.pose.orientation.y,
            odomData.pose.pose.orientation.z,
            odomData.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
#        print "Position_orientation: \n", euler[2]
    else :
        enable_task_manager=False
        print "-----"
        print "end"
        print "-----"

def originCallback(originData):
    global origin_utm_lon, origin_utm_lat, origin_set
    if not origin_set :
        origin_utm_lon = originData.x
        origin_utm_lat = originData.y
        origin_set=True

def task_manager():

    global pub, path, enable_task_manager, enable_task, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, tour_filename, threshold_waypoint, task_name, origin_utm_lon, origin_utm_lat, origin_lon, origin_lat, origin_set

    #ROS init
    rospy.init_node('task_manager', anonymous=True)

    if rospy.has_param('~reference'):
        reference=rospy.get_param('~reference')
    else :
        reference=('geo') #local or geo

    if rospy.has_param('~tour'):
        tour_filename=rospy.get_param('~tour')

    if rospy.has_param('~odometry_topic'):
        odometry_topic=rospy.get_param('~odometry_topic')
    else :
        odometry_topic=('/odom_gps')

    if rospy.has_param('~enable_task'):
        enable_task=rospy.get_param('~enable_task')
    else :
        enable_task=(True)

    if rospy.has_param('~task_name'):
        task_name=rospy.get_param('~task_name')
    else :
        task_name=('scanning')

    if rospy.has_param('~map_points'):
        map_points=rospy.get_param('~map_points')
    else :
#        map_points=('virtual_map_points_demo')
        map_points=('YoungGFP')

    if rospy.has_param('~threshold'):
        threshold=rospy.get_param('~threshold')

    if rospy.has_param('~threshold_waypoint'):
        threshold_waypoint=rospy.get_param('~threshold_waypoint')

    if rospy.has_param('~initial_waypoint_id'):
        waypoint_id=rospy.get_param('~initial_waypoint_id')
	waypoint_id=waypoint_id-1

    #map
    with open(path+ map_points+'.csv', 'rb') as csvfile:
        mapData = list(csv.reader(csvfile))	

    print map_points
    print mapData

    #cast map to float
    map_coordinates = [[0 for x in range(3)] for y in range(len(mapData))]

    utmProj = Proj("+proj=utm +zone=33N, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    if reference == 'geo':

        rospy.Subscriber('/ekf_slam_node/origin', Point, originCallback)

        #main loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not origin_set:
            rate.sleep() 

        #origin_utm_lon, origin_utm_lat = utmProj(origin_lon, origin_lat)

    i=0
    for elem in mapData:
        if reference == 'geo':
            utm_elem=[0, 0];
            utm_elem[0], utm_elem[1]= utmProj(float(elem[2]),float(elem[1]))
            map_coordinates[i][0] = utm_elem[0]-origin_utm_lon
            map_coordinates[i][1] = utm_elem[1]-origin_utm_lat
            map_coordinates[i][2] = float(elem[0])
        else :
            map_coordinates[i][0] = float(elem[0])
            map_coordinates[i][1] = float(elem[1])
            map_coordinates[i][2] = float(elem[2])
        i=i+1

    print "mapData: ", mapData
    print "map_coordinates", map_coordinates

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

    waypoint.pose.pose.position.x=map_coordinates[waypoint_id][0]
    waypoint.pose.pose.position.y=map_coordinates[waypoint_id][1]

    quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint_data[waypoint_id][1])
    waypoint.pose.pose.orientation.x = quaternion[0]
    waypoint.pose.pose.orientation.y = quaternion[1]
    waypoint.pose.pose.orientation.z = quaternion[2]
    waypoint.pose.pose.orientation.w = quaternion[3]

    new_waypoint = False

    pub = rospy.Publisher('/command/pose', Odometry, queue_size=1)
    rospy.Subscriber(odometry_topic, Odometry, odomCallback)
    s_glob = rospy.Service('run_task_manager', Empty, runTaskManagerService)

    #main loop
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if enable_task_manager :
            pub.publish(waypoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        task_manager()
    except rospy.ROSInterruptException:
        pass

