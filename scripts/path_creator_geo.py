#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
import csv
import math
import numpy
import time
import tf
import os
from os.path import expanduser
from sherpa_ros.srv import SavePoint,SavePointResponse


global pub, path, waypoint_id, tour_filename, map_filename, geo_filename, writer, current_odom, gps_data

default_workspace_value=expanduser("~")+'/catkin_ws'
path = os.getenv('ROS_WORKSPACE', default_workspace_value)+'/src/sherpa_ros/scripts/path_creator'

geo_filename='geo_test'
tour_filename='tour_test'
map_filename='map_test'
waypoint_id = 1

def runSavePointService(data):

    global path, waypoint_id, writer, current_odom

#    with open(path+'/'+tour_filename+'.csv', 'w') as csvfile:   
#        writer = csv.DictWriter(csvfile)
#        writer.writerow({'Baked', 'Beans'})
#        writer.writerow({'Bakedb', 'Beans'})
#        print "position " " service called"



#    with open(path+'/'+tour_filename+'.csv', 'w') as csvfile:
#        fieldnames = ['first_name', 'last_name']
#        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

#        writer.writeheader()
#        writer.writerow({'first_name': 'Baked', 'last_name': 'Beans'})
#        writer.writerow({'first_name': 'Lovely', 'last_name': 'Spam'})
#        writer.writerow({'first_name': 'Wonderful', 'last_name': 'Spam'})

#    with open('eggs.csv', 'wb') as csvfile:
#        spamwriter = csv.writer(csvfile, delimiter=' ',
#                                escapechar=' ', quoting=csv.QUOTE_NONE)
#        spamwriter.writerow(['Spam'] * 5 + ['Baked Beans'])
#        spamwriter.writerow(['Spam', 'Lovely Spam', 'Wonderful Spam'])

    with open(path+'/'+geo_filename+'.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow([waypoint_id, gps_data.latitude,gps_data.longitude])

    with open(path+'/'+map_filename+'.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow([current_odom.x,current_odom.y,waypoint_id])

    with open(path+'/'+tour_filename+'.csv', 'a') as f:
        writer = csv.writer(f)
        scan = data.scan[0] | data.scan[1] | data.scan[2] | data.scan[3]
        writer.writerow([waypoint_id,current_odom.z,scan,data.scan[0],data.scan[1],data.scan[2],data.scan[3]])


    waypoint_id=waypoint_id+1

    result = SavePointResponse()
    result.result = "done"

    return result

def gpsCallback(gpsData):
    global gps_data
    gps_data=gpsData

def odomCallback(odomData):

    global tour_filename, current_odom, waypoint_id

    print "-----"
    print "tour: ", tour_filename
    print "Waypoint_ID: ", waypoint_id
    print "Position_position: \n", odomData.pose.pose.position


    current_odom.x=odomData.pose.pose.position.x
    current_odom.y=odomData.pose.pose.position.y

    quaternion = (
        odomData.pose.pose.orientation.x,
        odomData.pose.pose.orientation.y,
        odomData.pose.pose.orientation.z,
        odomData.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print "Position_orientation: \n", euler[2]

    current_odom.z=euler[2]

def tour_creator():

    global pub, path, tour_filename, map_filename, geo_filename, writer, current_odom,gps_data

    #ROS init
    rospy.init_node('tour_creator', anonymous=True)

    if rospy.has_param('~tour'):
        tour_filename=rospy.get_param('~tour')

    if rospy.has_param('~map'):
        map_filename=rospy.get_param('~map')

    if rospy.has_param('~geo'):
        geo_filename=rospy.get_param('~geo')

    if rospy.has_param('~odometry_topic'):
        odometry_topic=rospy.get_param('~odometry_topic')
    else :
        odometry_topic=('/ekf_slam_node/slam_odom_geo')

    if rospy.has_param('~gps_topic'):
        gps_topic=rospy.get_param('~gps_topic')
    else :
        gps_topic=('/ekf_slam_node/slam_fix')

    current_odom=Point()
    gps_data=NavSatFix()

    rospy.Subscriber(odometry_topic, Odometry, odomCallback)
    rospy.Subscriber(gps_topic, NavSatFix, gpsCallback)
    s_glob = rospy.Service('save_point', SavePoint, runSavePointService)

    # clean file
    with open(path+'/'+tour_filename+'.csv', 'w') as f:
        writer = csv.writer(f)
    with open(path+'/'+map_filename+'.csv', 'w') as f:
        writer = csv.writer(f)
    with open(path+'/'+geo_filename+'.csv', 'w') as f:
        writer = csv.writer(f)

    #main loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        tour_creator()
    except rospy.ROSInterruptException:
        pass

