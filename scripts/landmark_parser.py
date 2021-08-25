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
        suckers_landmarks_file = path+'suckers/'+area_file_name+'_landmark'+'.csv'
        suckers_map_file = path+'suckers/'+area_file_name+'_map'+'.csv'
        suckers_tour_file = path+'suckers/'+area_file_name+'_tour'+'.csv'
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
                    print "WARNING - LANDMARK ALREADY ASSOCIATED WITH SUCKER"
                    id_landmark_in_list = numpy.where(landmark_selected_ids==min_landmark_id)
                    id_ld = id_landmark_in_list[0][0]
                    # check if the new sucker is close to the one associated with the landmark. If so, add the contribution to the previous one,
                    # otherwise create a new entry in the list
                    # NB: in the rare case there are 3 or more suckers associated to the same landmark and far enough from each other, the check
                    # with respect to new potential neighbors is made only with the first one in the list [yes, it is me being lazy but this 
                    # case should never occour]
                    old_suckerCoords = numpy.array([[suckers_landmarks_data[id_ld][4],suckers_landmarks_data[id_ld][5]]])
                    # compute distance between the two bushes. If they are close than 0.5 meters associated them together
                    suckers_pair_distance = distance.cdist(old_suckerCoords,suckerCoords,'euclidean')
                    if suckers_pair_distance < 0.5:
                        print "The two suckers are close enough. Summing contributions."
                        # add area, mL, and seconds
                        suckers_landmarks_data[id_ld][1] = suckers_landmarks_data[id_ld][1] + sucker[1]
                        suckers_landmarks_data[id_ld][2] = suckers_landmarks_data[id_ld][2] + sucker[2]
                        suckers_landmarks_data[id_ld][3] = suckers_landmarks_data[id_ld][3] + sucker[3]
                    else:
                        print "The two suckers are not close enough. Creating new entry."
                        # add a new original one without adding the id to landmark_selected_ids
                        sucker_item = numpy.reshape(numpy.append(sucker,[landmarksCoords_XYZ[min_landmark_id][0], landmarksCoords_XYZ[min_landmark_id][1], landmarksCoords_XYZ[min_landmark_id][2]]),(1,10))
                        suckers_landmarks_data = numpy.append(suckers_landmarks_data,sucker_item,axis=0)
                else:
                    # store landmark id and seconds for sprayer
                    landmark_selected_ids = numpy.append(landmark_selected_ids,min_landmark_id)
                    sucker_item = numpy.reshape(numpy.append(sucker,[landmarksCoords_XYZ[min_landmark_id][0], landmarksCoords_XYZ[min_landmark_id][1], landmarksCoords_XYZ[min_landmark_id][2]]),(1,10))
                    suckers_landmarks_data = numpy.append(suckers_landmarks_data,sucker_item,axis=0)
                    
        # Write data to output file as: [sucker_info, x_landmark, y_landmark, z_landmark]
        with open(suckers_landmarks_file, mode='w') as output:
            output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            count = 1
            for row in suckers_landmarks_data:
                row[0] = count
                output_writer.writerow(row)
                count = count+1
                
        # Read target list
        mission_target_file = 'mission_processed.json'
        with open(mission_target_file, 'r') as mission_json_file:
            mission_data = json.load(mission_json_file)
            final_targets = mission_data['targets']

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
                    dict_tree[d['id']] = numpy.array([utm_elem[1]-origin_utm_lat, utm_elem[0]-origin_utm_lon])
            
        # Parse target list and suckers_landmarks_data
        threshold_tree_distance = 1.0
        tree_landmarks_data = numpy.empty((0,10))
        landmarksCoords = suckers_landmarks_data[:,7:9]
        selected_trees = []
        for tree in final_targets:
            tree_coords = [dict_tree[tree]]
            # Compute distance to landmarks for the tree
            tree_landmarks_dist = distance.cdist(landmarksCoords,tree_coords,'euclidean')
            # Find the closest
            min_landmark_id = numpy.argmin(tree_landmarks_dist)
            # If the landmark is within "threshold" distance from the tree then add it to the list
            if tree_landmarks_dist[min_landmark_id] < threshold_tree_distance:
                tree_landmarks_data = numpy.append(tree_landmarks_data,numpy.reshape(suckers_landmarks_data[min_landmark_id],(1,10)),axis=0)
                selected_trees.append(tree)
        # in tree_landmarks_data abbiamo i final target con quantitivo e in selected_trees c'e' la lista finale degli alberi scelti che hanno un landmark
        # che ricade nella soglia threshold_tree_distance

        rotation_map = numpy.array([[1,1],[1,1]])
        rotation_theta_zero = numpy.dot(rotation_map,numpy.array([[0.9995, -0.0303], [0.0303, 0.9995]]))
        rotation_theta_pi = numpy.dot(rotation_map,numpy.array([[-0.9995, 0.0303], [-0.0303, -0.9995]]))
        delta_col = 2.5
        delta_row = 2.5
        delta_vec = numpy.array([[-delta_col/2],[delta_row/2]])

        # load tour data
        with open('tour.csv', 'rb') as csvfile:
            tourData = list(csv.reader(csvfile))
        # convert to float array
        tour_array = numpy.array([[float(x) for x in data] for data in tourData])
        # load map data
        with open('map.csv', 'rb') as csvfile:
            mapData = list(csv.reader(csvfile))
        # convert to float array
        map_array = numpy.array([[float(x) for x in data] for data in mapData])
        map_array_coords = map_array[:,1:3]
        last_id_entry_map_array = map_array[-1][0]

        tree_count_id = 0
        new_entries_flag = false
        map_entries = numpy.empty((0,3))
        tour_entries = numpy.empty((0,7))

        for tree in selected_trees:
            tree_coords = numpy.reshape(dict_tree[tree],(2,1))
            point_A = tree_coords + rotation_theta_zero.dot(delta_vec)
            point_B = tree_coords + rotation_theta_pi.dot(delta_vec)
            # check points A and B with reference of map
            dist_A = distance.cdist(map_array_coords,numpy.reshape(point_A,(1,2)))
            dist_B = distance.cdist(map_array_coords,numpy.reshape(point_B,(1,2)))
            min_id_A = numpy.argmin(dist_A)
            min_id_B = numpy.argmin(dist_B)
            map_id_min_A = map_array[min_id_A,0]
            map_id_min_B = map_array[min_id_B,0]
            # map_id_min_A/B contains the ID of the entry on map -> now look for this ID in the tour file
            # point_A must correspond to orientation close to 0 whereas point B close to -pi (using >0 and <0 to check it)
            found_point_A = false
            found_point_B = false
            if tour_array[map_id_min_A][1] > 0:
                found_point_A = true
            if tour_array[map_id_min_B][1] < 0:
                found_point_B = true
            if found_point_A or found_point_B:
                if found_point_A and found_point_B:
                    # both points are part of the tour, select the one closer to the sucker
                    suckerCoords = tree_landmarks_data[tree_count_id][4:6]
                    sucker_dist_to_A = distance.cdist(suckerCoords,numpy.reshape(point_A,(1,2)))
                    sucker_dist_to_B = distance.cdist(suckerCoords,numpy.reshape(point_B,(1,2)))
                    if sucker_dist_to_A < sucker_dist_to_B:
                        # point_A is closer
                        tour_array[map_id_min_A][2] = 3
                    else:
                        # point_B is closer
                        tour_array[map_id_min_B][2] = 3
                elif found_point_A:
                    # adjust entry of tour for point A
                    tour_array[map_id_min_A][2] = 3
                else:
                    # adjust entry of tour for point B
                    tour_array[map_id_min_B][2] = 3
            else:
                print "No tour point has been found. New one will be added to map and tour."
                last_id_entry_map_array = last_id_entry_map_array+1
                map_entry = numpy.array([last_id_entry_map_array, point_A[0], point_A[1]])
                tour_entry = numpy.array([last_id_entry_map_array, 0.0303346732, 3, 0, 0, 0, 0])
                # add to list
                map_entries = numpy.append(map_entries,numpy.reshape(map_entry,(1,3)),axis=0)
                tour_entries = numpy.append(tour_entries,numpy.reshape(tour_entry,(1,7)),axis=0)
                new_entries_flag = true
            tree_count_id = tree_count_id + 1

        # store final data
        if new_entries_flag:
            map_final = numpy.append(map_array,map_entries,axis=0)
            tour_final = numpy.append(tour_array,tour_entries,axis=0)
        else:
            map_final = map_array
            tour_final = tour_array

        # write new map file
        with open(suckers_map_file, mode='w') as output:
            output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for row in map_final:
                output_writer.writerow(row)

        # write new tour file
        with open(suckers_tour_file, mode='w') as output:
            output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for row in tour_final:
                output_writer.writerow(row)


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

