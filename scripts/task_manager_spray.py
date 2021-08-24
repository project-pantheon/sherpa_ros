#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Header

import csv
import math
import numpy
import time
import tf
import os

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotnik_msgs.srv import set_digital_output

from os.path import expanduser


global pub, path, enable_task_manager, enable_spray, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, new_waypoint, tour_filename, spray_command

default_workspace_value=expanduser("~")+'/catkin_ws'
path = os.getenv('ROS_WORKSPACE', default_workspace_value)+'/src/sherpa_ros/scripts/tours/'

tour_filename='ackermann_tour'

enable_task_manager = False
enable_spray = True
new_waypoint = True
waypoint_id = 0
threshold=0.2


tmp=1

def updateWaypointFromTour():

    global waypoint, waypoint_id, map_coordinates, waypoint_data, new_waypoint

    if new_waypoint :
        for elem in map_coordinates:
            if elem[2] == waypoint_data[waypoint_id][0]:
                waypoint.pose.pose.position.x=elem[0]
                waypoint.pose.pose.position.y=elem[1]

                quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint_data[waypoint_id][1])
                waypoint.pose.pose.orientation.x = quaternion[0]
                waypoint.pose.pose.orientation.y = quaternion[1]
                waypoint.pose.pose.orientation.z = quaternion[2]
                waypoint.pose.pose.orientation.w = quaternion[3]

                new_waypoint = False
                break


def runTaskManagerService(call):

    global enable_task_manager

    enable_task_manager = True
    print "task_manager service called"

    return EmptyResponse()


def odomCallback(odomData):

    global waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, enable_task_manager, enable_spray, new_waypoint, tour_filename, tmp

    # distance
    a = numpy.array((odomData.pose.pose.position.x, odomData.pose.pose.position.y))
    b = numpy.array((waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
    dist = numpy.linalg.norm(a-b)

    # check distance
    if waypoint_id+1<max_waypoint_id and enable_task_manager :
        if dist<threshold :

            #time.sleep(5)
            
            # check if spray action is required
            if waypoint_data[waypoint_id][2] >= 1 :

                if enable_spray :

                    #lock spray operations
                    print("spray")
                    

                    
#                    #spray calls
#                    rospy.wait_for_service('/spray_tree')
#                    try:
#                        spray_tree_client = rospy.ServiceProxy('/spray_tree', SprayTree)


#                        spray_type = [waypoint_data[waypoint_id][3], waypoint_data[waypoint_id][4], waypoint_data[waypoint_id][5], waypoint_data[waypoint_id][6] ]


#                        resp = spray_tree_client(spray_type)
#                    except rospy.ServiceException, e:
#                        print "Task Manager: SprayTree service call failed: %s"%e

                else :
                    time.sleep(waypoint_data[waypoint_id][2]*5)

                #set spray done
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

    global pub, path, enable_task_manager, enable_spray, waypoint, waypoint_id, max_waypoint_id, threshold, map_coordinates, waypoint_data, tour_filename, spray_command

    #ROS init
    rospy.init_node('task_manager', anonymous=True)

    if rospy.has_param('~tour'):
        tour_filename=rospy.get_param('~tour')

    if rospy.has_param('~odometry_topic'):
        odometry_topic=rospy.get_param('~odometry_topic')
    else :
        odometry_topic=('/base/odom')

    if rospy.has_param('~enable_spray'):
        enable_spray=rospy.get_param('~enable_spray')
    else :
        enable_spray=(True)

    if rospy.has_param('~map_points'):
        map_points=rospy.get_param('~map_points')
    else :
        map_points=('ackermann_map')

    if rospy.has_param('~threshold'):
        threshold=rospy.get_param('~threshold')

    enable_spray

    #map
    with open(path+ map_points+'.csv', 'rb') as csvfile:
        mapData = list(csv.reader(csvfile))	

    print map_points
    print mapData

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

    waypoint.pose.pose.position.x=map_coordinates[waypoint_id][0]
    waypoint.pose.pose.position.y=map_coordinates[waypoint_id][1]

    quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint_data[waypoint_id][1])
    waypoint.pose.pose.orientation.x = quaternion[0]
    waypoint.pose.pose.orientation.y = quaternion[1]
    waypoint.pose.pose.orientation.z = quaternion[2]
    waypoint.pose.pose.orientation.w = quaternion[3]

    new_waypoint = False

    spray_command = JointTrajectory()
    spray_command.header = Header()
    spray_command.joint_names.append("rbsherpa_hl_torso_pantilt_pan_joint")
    spray_command.joint_names.append("rbsherpa_hl_torso_pantilt_tilt_joint")
    point = JointTrajectoryPoint()
    point.positions.append(0.5)
    point.positions.append(0.5)
    point.time_from_start=rospy.Duration(0.1)
    spray_command.points.append(point)

    print (spray_command)


    pub = rospy.Publisher('/command/pose', Odometry, queue_size=1) #local planner
    pubSpray = rospy.Publisher('/pantilt/pantilt_position_joint_trajectory_controller/command', JointTrajectory, queue_size=1)
    rospy.Subscriber(odometry_topic, Odometry, odomCallback)
    s_glob = rospy.Service('run_task_manager', Empty, runTaskManagerService)

    #main loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pubSpray.publish(spray_command)
        if enable_task_manager :
            pub.publish(waypoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        task_manager()
    except rospy.ROSInterruptException:
        pass

