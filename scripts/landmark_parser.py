#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty, EmptyResponse
from scanning_controller.srv import ScanningTree, ScanningTreeResponse, ScanningTreeRequest, OffsetSet, OffsetSetResponse, OffsetSetRequest
from rm3_ackermann_controller.srv import ActivateController, ActivateControllerResponse, ActivateControllerRequest
from pyproj import Proj
from scipy.spatial import distance

import csv
import math
import numpy
import time
import tf
import os
from os.path import expanduser

from paramiko import SSHClient
from scp import SCPClient

global threshold, landmarks, path, origin_utm_lon, origin_utm_lat, origin_set

threshold=1.0;

default_workspace_value=expanduser("~")+'/catkin_ws'
path = os.getenv('ROS_WORKSPACE', default_workspace_value)+'/src/sherpa_ros/scripts/'

def landmarksCallback(landmarksData):

    global landmarks
    landmarks=landmarksData

def parsingService(call):

    global origin_utm_lon, origin_utm_lat

    # load landmarks data
    landmarksCoords = numpy.empty((0,2))
    landmarksCoords_XYZ = numpy.empty((0,3))
    for point in landmarks.points:
        landmarksCoords = numpy.append(landmarksCoords,[[point.x, point.y]],axis=0)
        landmarksCoords_XYZ = numpy.append(landmarksCoords,[[point.x, point.y, point.z]],axis=0)

    #recupero del file prodotto 
    #determina percorso
    if rospy.has_param('/mesh_filter/area_path'):
        area_file_path=rospy.get_param('/mesh_filter/area_path')
    else :
        area_file_path=('/home/newline/catkin_ws/src/mesh_filter/area/')
    if rospy.has_param('/mesh_filter/filename'):
        area_file_name = rospy.get_param('/mesh_filter/filename')
        area_suckers_file = area_file_path+area_file_name+'.txt'
        suckers_area_tour_file = path+'suckers/'+area_file_name+'_landmark'+'.csv'
        # create ssh and scp objects
        ssh = SSHClient()
        ssh.load_system_host_keys()
        ssh.connect("10.10.1.125",22,"newline","pantheon.jetson")
        scp = SCPClient(ssh.get_transport())
        # retrieve file
        if not os.path.isdir(path+'suckers'):
            os.mkdir(path+'suckers')
        suckers_file_path = path+'suckers/'+area_file_name+'.txt'
        scp.get(area_suckers_file,suckers_file_path)
        # close ssh and scp
        scp.close()
        ssh.close()
        # parse del file
        suckersData = numpy.genfromtxt(suckers_area_origin_file,delimiter=',')

        # list of landmarks and suckers
        landmark_selected_ids = numpy.array([])
        sprayer_seconds_list = numpy.array([])
        suckers_landmarks_data = numpy.empty((0,10))

        for sucker in suckersData:
            # Store coords of sucker
            suckerCoords = numpy.array([[sucker[4], sucker[5]]])
            # Compute distance to landmarks for the sucker
            sucker_landmarks_dist = distance.cdist(landmarksCoords,suckerCoords,'euclidean')
            # Find the closest
            min_landmark_id = numpy.argmin(sucker_landmarks_dist)
            print "ID Landmark associated to sucker: ", min_landmark_id
            print "Area: ", sucker[1]
            print "Sprayer sec: ", sucker[3]
            # If the landmark is within "threshold" distance from the sucker then add it to the list
            if sucker_landmarks_dist[min_landmark_id] < threshold:
                # Check if the landmark has been already selected, if so add the relative seconds for the sprayer
                if min_landmark_id in landmark_selected_ids:
                    print "WARNING - LANDMARK ALREADY ASSOCIATED WITH SUCKER - Summing contributions"
                    id_landmark_in_list = numpy.where(landmark_selected_ids==min_landmark_id)
                    id_ld = id_landmark_in_list[0][0]
                    # add area, mL, and seconds
                    suckers_landmarks_data[id_ld][1] = suckers_landmarks_data[id_ld][1] + sucker[1]
                    suckers_landmarks_data[id_ld][2] = suckers_landmarks_data[id_ld][2] + sucker[2]
                    suckers_landmarks_data[id_ld][3] = suckers_landmarks_data[id_ld][3] + sucker[3]
                else:
                    # store landmark id and seconds for sprayer
                    landmark_selected_ids = numpy.append(landmark_selected_ids,min_landmark_id)
                    sucker_item = numpy.reshape(numpy.append(sucker,[landmarksCoords_XYZ[min_landmark_id][0], landmarksCoords_XYZ[min_landmark_id][1], landmarksCoords_XYZ[min_landmark_id][2]]),(1,10))
                    suckers_landmarks_data = numpy.append(suckers_landmarks_data,sucker_item,axis=0)
                    
        # Write data to output file as: [sucker_info, x_landmark, y_landmark, z_landmark]
        with open(suckers_area_tour_file, mode='w') as output:
            output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            count = 1
            for row in suckers_landmarks_data:
                row[0] = count
                output_writer.writerow(row)
                count = count+1
                
        # Read target list
        final_targets = ["Yo_A4", "Yo_A5"]

        # Local coordinates of trees
        utmProj = Proj("+proj=utm +zone=33N, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        filename_geodata_json = 'geoObjects.json'
        with open(filename_geodata_json, 'rb') as geo_json_file:
            data = json.load(geo_json_file)
            dict_tree = {}
            for d in data:
                if d['geometry']['type']=='Point':
                    temp_coords = d['geometry']['coordinates']
                    utm_elem=[0, 0];
                    utm_elem[0], utm_elem[1]= utmProj(float(temp_coords[0]),float(temp_coords[1]))
                    dict_tree[d['id']] = numpy.array([utm_elem[0]-origin_utm_lon, utm_elem[1]-origin_utm_lat])
            
        # Parse target list and suckers_landmarks_data
        threshold_tree_distance = 1.0
        tree_landmarks_data = numpy.empty((0,10))
        landmarksCoords = suckers_landmarks_data[:,7:9]
        for tree in final_targets:
            tree_coords = dict_tree[tree]
            # Compute distance to landmarks for the tree
            tree_landmarks_dist = distance.cdist(landmarksCoords,tree_coords,'euclidean')
            # Find the closest
            min_landmark_id = numpy.argmin(tree_landmarks_dist)
            # If the landmark is within "threshold" distance from the tree then add it to the list
            if tree_landmarks_dist[min_landmark_id] < threshold_tree_distance:
                tree_landmarks_data = numpy.append(tree_landmarks_data,suckers_landmarks_data[min_landmark_id],axis=0)
        # in tree_landmarks_data abbiamo i final target con quantitivo



    else :
        print "Could not retrieve suckers' area"


    print "landmark_parser Parsing service called"

    return EmptyResponse()

def tourService(call):


    print "landmark_parser Tour service called"

    return EmptyResponse()

def originCallback(originData):
    global origin_utm_lon, origin_utm_lat, origin_set
    if not origin_set :
        origin_utm_lon = originData.x
        origin_utm_lat = originData.y
        origin_set=True    

def landmark_parser():

    global threshold

    #ROS init
    rospy.init_node('landmark_parser', anonymous=True)

    if rospy.has_param('~landmarks_topic'):
        landmarks_topic=rospy.get_param('~landmarks_topic')
    else :
        landmarks_topic=('/ekf_slam_node/visualization_marker_all_map')

    landmarks=Marker()

    rospy.Subscriber(landmarks_topic, Marker, landmarksCallback)
    rospy.Subscriber('/ekf_slam_node/origin', Point, originCallback)
    s_parsing = rospy.Service('/parsing', Empty, parsingService)
    s_tour = rospy.Service('/tour', Empty, tourService)

    rospy.spin()

if __name__ == '__main__':
    try:
        landmark_parser()
    except rospy.ROSInterruptException:
        pass

