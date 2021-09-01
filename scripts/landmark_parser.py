#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty, EmptyResponse
from rm3_ackermann_controller.srv import ActivateController, ActivateControllerResponse, ActivateControllerRequest
from pyproj import Proj
from scipy.spatial import distance

import csv
import json
import math
import numpy
import time
import tf
import os
from os.path import expanduser

from paramiko import SSHClient
from scp import SCPClient

global threshold, landmarks, path, origin_utm_lon, origin_utm_lat, origin_set

threshold_sucker_distance = 1.5
threshold_tree_distance = 1.5

default_workspace_value=expanduser("~")+'/catkin_ws'
path = os.getenv('ROS_WORKSPACE', default_workspace_value)+'/src/sherpa_ros/scripts/'

def landmarksCallback(landmarksData):

    global landmarks
    landmarks=landmarksData

def parsingService(call):

    global origin_utm_lon, origin_utm_lat, landmarks

    sucker_height = 0.2

    # load landmarks data
    landmarksCoords = numpy.empty((0,2))
    landmarksCoords_XYZ = numpy.empty((0,3))
    for point in landmarks.points:
        landmarksCoords = numpy.append(landmarksCoords,numpy.array([[point.x, point.y]]),axis=0)
        landmarksCoords_XYZ = numpy.append(landmarksCoords_XYZ,numpy.array([[point.x, point.y, sucker_height]]),axis=0)
        
    print "LANDMARKS\n", landmarksCoords, '\n'

    #recupero del file prodotto 
    #determina percorso
    if rospy.has_param('/mesh_filter/area_path'):
        area_file_path=rospy.get_param('/mesh_filter/area_path')
    else :
        area_file_path=('/home/newline/catkin_ws/src/mesh_filter/area/')
    #if rospy.has_param('/mesh_filter/filename'):
    if True:
        #area_file_name = rospy.get_param('/mesh_filter/filename')
        area_file_name = 'suckers_31_08'
        area_suckers_file = area_file_path+area_file_name+'.txt'
        suckers_landmarks_file = path+'suckers/'+area_file_name+'_landmark'+'.csv'
        suckers_map_file = path+'current_mission/spray_map'+'.csv'
        suckers_tour_file = path+'current_mission/spray_tour'+'.csv'
        # create ssh and scp objects
        ssh = SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
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
        suckersData = numpy.genfromtxt(suckers_file_path,delimiter=',')

        # list of landmarks and suckers
        landmark_selected_ids = numpy.array([])
        suckers_landmarks_data = numpy.empty((0,10))
        
        # if array has at least two suckers
        if len(suckersData.shape)>1 :
            for sucker in suckersData:
                # Store coords of sucker
                suckerCoords = numpy.array([[sucker[4], sucker[5]]])
                # Compute distance to landmarks for the sucker
                sucker_landmarks_dist = distance.cdist(landmarksCoords,suckerCoords,'euclidean')
                # Find the closest
                min_landmark_id = numpy.argmin(sucker_landmarks_dist)
                print "ID Landmark nearest to sucker: ", min_landmark_id+1
                print "Distance from selected landmark: ", sucker_landmarks_dist[min_landmark_id]
                print "Area: ", sucker[1]
                print "Sprayer sec: ", sucker[3], '\n'
                # If the landmark is within "threshold" distance from the sucker then add it to the list
                if sucker_landmarks_dist[min_landmark_id] < threshold_sucker_distance:
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
        
        else:
            sucker = suckersData
            suckerCoords = numpy.array([[sucker[4], sucker[5]]])
            sucker_landmarks_dist = distance.cdist(landmarksCoords,suckerCoords,'euclidean')
            min_landmark_id = numpy.argmin(sucker_landmarks_dist)
            print "ID Landmark nearest to sucker: ", min_landmark_id+1
            print "Distance from selected landmark: ", sucker_landmarks_dist[min_landmark_id]
            print "Area: ", sucker[1]
            print "Sprayer sec: ", sucker[3], '\n'
            if sucker_landmarks_dist[min_landmark_id] < threshold_sucker_distance:
                landmark_selected_ids = numpy.append(landmark_selected_ids,min_landmark_id)
                sucker_item = numpy.reshape(numpy.append(sucker,[landmarksCoords_XYZ[min_landmark_id][0], landmarksCoords_XYZ[min_landmark_id][1], landmarksCoords_XYZ[min_landmark_id][2]]),(1,10))
                suckers_landmarks_data = numpy.append(suckers_landmarks_data,sucker_item,axis=0)
                
        # Suckers Landmark data ready
        print "Suckers LANDMARKS:\n", suckers_landmarks_data, '\n'
        
        if suckers_landmarks_data.size > 0:      
            # Write data to output file as: [sucker_info, x_landmark, y_landmark, z_landmark]
            with open(suckers_landmarks_file, mode='w') as output:
                output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                count = 1
                for row in suckers_landmarks_data:
                    row[0] = count
                    output_writer.writerow(row)
                    count = count+1
                    
            # Read target list
            mission_target_file = path+'current_mission/mission.json'
            with open(mission_target_file, 'r') as mission_json_file:
                mission_data = json.load(mission_json_file)
                final_targets = mission_data['targets']

            # Local coordinates of trees
            utmProj = Proj("+proj=utm +zone=33N, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
            filename_geodata_json = path+'current_mission/geoObjects.json'
            with open(filename_geodata_json, 'rb') as geo_json_file:
                data = json.load(geo_json_file)
                dict_tree = {}
                for d in data:
                    if d['geometry']['type']=='Point':
                        temp_coords = d['geometry']['coordinates']
                        utm_elem=[0, 0];
                        utm_elem[0], utm_elem[1]= utmProj(float(temp_coords[0]),float(temp_coords[1]))
                        dict_tree[d['id']] = numpy.array([utm_elem[0]-origin_utm_lon, utm_elem[1]-origin_utm_lat])
                
		    # Parse target list and suckers_landmarks_data
            tree_landmarks_data = numpy.empty((0,10))
            landmarksCoords = suckers_landmarks_data[:,7:9]
            selected_trees = []
            for tree in final_targets:
                print "Evaluating tree ", tree
                tree_coords = [dict_tree[tree]]
                print "Tree coords: ", tree_coords
                # Compute distance to landmarks for the tree
                tree_landmarks_dist = distance.cdist(landmarksCoords,tree_coords,'euclidean')
                # Find the closest
                min_landmark_id = numpy.argmin(tree_landmarks_dist)
                print "Distance tree-landmark: ", tree_landmarks_dist[min_landmark_id]
                # If the landmark is within "threshold" distance from the tree then add it to the list
                if tree_landmarks_dist[min_landmark_id] < threshold_tree_distance:
                    tree_landmarks_data = numpy.append(tree_landmarks_data,numpy.reshape(suckers_landmarks_data[min_landmark_id],(1,10)),axis=0)
                    selected_trees.append(tree)
            
            print "\nTree landmarks: ", tree_landmarks_data, '\n'
            print "Selected trees: ", selected_trees, '\n'
            
            # in tree_landmarks_data abbiamo i final target con quantitivo e in selected_trees c'e' la lista finale degli alberi scelti che hanno un landmark che ricade nella soglia threshold_tree_distance

            field_id_name = mission_data['location']
            map_config_file = path+'maps/maps_config.json'
            map_data_found = False
            # find map data from map_config_file
            with open(map_config_file, 'rb') as map_config_json_file:
                map_config_data = json.load(map_config_json_file)
                for d in map_config_data:
                    if d['id']==field_id_name:
                        map_orientation = d['field']['orientation']
                        delta_rows = d['field']['delta_rows']
                        delta_cols = d['field']['delta_cols']
                        map_data_found = True
            if not map_data_found:
                map_orientation = 0.0303346732
                delta_rows = 4
                delta_cols = 3.5
            
            # compute rotation matrices data
            rotation_map = numpy.array([[numpy.cos(map_orientation),-numpy.sin(map_orientation)],[numpy.sin(map_orientation),numpy.cos(map_orientation)]])
            rotation_theta_zero = numpy.dot(rotation_map,numpy.array([[0.9995, -0.0303], [0.0303, 0.9995]]))
            rotation_theta_pi = numpy.dot(rotation_map,numpy.array([[-0.9995, 0.0303], [-0.0303, -0.9995]]))
            
            # compute delta vector
            delta_vec = numpy.array([[-delta_cols/2],[delta_rows/2]])

            # load tour data
            with open(path+'current_mission/tour.csv', 'rb') as csvfile:
                tourData = list(csv.reader(csvfile))
            # convert to float array
            tour_array = numpy.array([[float(x) for x in data] for data in tourData])
            # remove info about 1 and 2 operations from tour data
            for index in range(len(tour_array)):
                tour_array[index][2] = 0
            
            print "TOUR DATA:\n", tour_array, '\n'
            
            # load map data
            with open(path+'current_mission/map.csv', 'rb') as csvfile:
                mapData = list(csv.reader(csvfile))
            # convert to float array
            map_array = numpy.array([[float(x) for x in data] for data in mapData])
            for index in range(len(map_array)):
                utm_elem=[0, 0];
                utm_elem[0], utm_elem[1] = utmProj(float(map_array[index][2]),float(map_array[index][1]))
                map_array[index][1] = utm_elem[0]-origin_utm_lon
                map_array[index][2] = utm_elem[1]-origin_utm_lat
                
            print "MAP DATA:\n", map_array, '\n'
            
            map_array_coords = map_array[:,1:3]
            last_id_entry_map_array = map_array[-1][0]

            tree_count_id = 0
            new_entries_flag = False
            map_entries = numpy.empty((0,3))
            tour_entries = numpy.empty((0,7))
            reduction_distance = 1.0
            threshold_grid = 1.0
            angle_threshold = 0.4363 # = 25 degrees, this threshold is used to discern the orientation of the tour 

            for tree in selected_trees:
                print "Evaluating Tree: ", tree, '\n'
                tree_coords = numpy.reshape(dict_tree[tree],(2,1))
                point_A = tree_coords + rotation_theta_zero.dot(delta_vec)
                point_B = tree_coords + rotation_theta_pi.dot(delta_vec)
                # check points A and B with reference of map
                dist_A = distance.cdist(map_array_coords,numpy.reshape(point_A,(1,2)))
                print "Dist A to tree:\n", dist_A, '\n'
                dist_B = distance.cdist(map_array_coords,numpy.reshape(point_B,(1,2)))
                print "Dist B to tree:\n", dist_B, '\n'
                min_id_A = numpy.argmin(dist_A)
                min_id_B = numpy.argmin(dist_B)
                # if the point A is close enough to the closest grid point
                if dist_A[min_id_A] < threshold_grid:
                    map_id_min_A = int(map_array[min_id_A,0])-1
                else:
                    map_id_min_A = -1
                # if the point B is close enough to the closest grid point
                if dist_B[min_id_B] < threshold_grid:
                    map_id_min_B = int(map_array[min_id_B,0])-1
                else:
                    map_id_min_B = -1
                    
                # map_id_min_A/B contains the ID of the entry on map -> now look for this ID in the tour file
                # if the ID is 0 then it was too far from the known grid points
                # point_A must correspond to orientation close to 0 whereas point B close to -pi (using >0 and <0 to check it)
                found_point_A = False
                found_point_B = False
                
                # check if the ID is in list and if the orientation is close to 0
                if map_id_min_A+1 in tour_array[:,0:1] and abs(tour_array[map_id_min_A][1]-0)<angle_threshold:
                    found_point_A = True
                # check if the ID is in list and if the orientation is close to pi
                if map_id_min_B+1 in tour_array[:,0:1] and abs(abs(tour_array[map_id_min_B][1])-numpy.pi)<angle_threshold:
                    found_point_B = True
                # store sucker coords ---- #EDIT NOW WE TAKE LANDMARK COORDS
                suckerCoords = tree_landmarks_data[tree_count_id][7:9]
                # evaluate if ID has been found
                if found_point_A or found_point_B:
                    delta_vec_closer = delta_vec * reduction_distance
                    point_A_closer = tree_coords + rotation_theta_zero.dot(delta_vec_closer)
                    point_B_closer = tree_coords + rotation_theta_pi.dot(delta_vec_closer)
                    if found_point_A and found_point_B:
                        # both points are part of the tour, select the one closer to the sucker
                        sucker_dist_to_A = distance.cdist(numpy.reshape(suckerCoords,(1,2)),numpy.reshape(point_A,(1,2)))
                        sucker_dist_to_B = distance.cdist(numpy.reshape(suckerCoords,(1,2)),numpy.reshape(point_B,(1,2)))
                        if sucker_dist_to_A < sucker_dist_to_B:
                            # point_A is closer
                            tour_array[map_id_min_A][1] = -0.7853981634 #-45 degrees
                            tour_array[map_id_min_A][2] = 3
                            # add information about sucker: x, y, 0.2, seconds of sprayer
                            tour_array[map_id_min_A][3] = suckerCoords[0]
                            tour_array[map_id_min_A][4] = suckerCoords[1]
                            tour_array[map_id_min_A][5] = sucker_height #fix 20 cm from ground
                            tour_array[map_id_min_A][6] = tree_landmarks_data[tree_count_id][3]
                            # update map with closer point
                            map_array[map_id_min_A][1] = point_A_closer[0]
                            map_array[map_id_min_A][2] = point_A_closer[1]
                        else:
                            # point_B is closer
                            tour_array[map_id_min_B][1] = 2.3561944901 #135 degrees
                            tour_array[map_id_min_B][2] = 3
                            # add information about sucker: x, y, 0.2, seconds of sprayer
                            tour_array[map_id_min_B][3] = suckerCoords[0]
                            tour_array[map_id_min_B][4] = suckerCoords[1]
                            tour_array[map_id_min_B][5] = sucker_height #fix 20 cm from ground
                            tour_array[map_id_min_B][6] = tree_landmarks_data[tree_count_id][3]
                            # update map with closer point
                            map_array[map_id_min_B][1] = point_B_closer[0]
                            map_array[map_id_min_B][2] = point_B_closer[1]
                    elif found_point_A:
                        # adjust entry of tour for point A
                        tour_array[map_id_min_A][1] = -0.7853981634 #-45 degrees
                        tour_array[map_id_min_A][2] = 3
                        # add information about sucker: x, y, 0.2, seconds of sprayer
                        tour_array[map_id_min_A][3] = suckerCoords[0]
                        tour_array[map_id_min_A][4] = suckerCoords[1]
                        tour_array[map_id_min_A][5] = sucker_height #fix 20 cm from ground
                        tour_array[map_id_min_A][6] = tree_landmarks_data[tree_count_id][3]
                        # update map with closer point
                        map_array[map_id_min_A][1] = point_A_closer[0]
                        map_array[map_id_min_A][2] = point_A_closer[1]
                    else:
                        # adjust entry of tour for point B
                        tour_array[map_id_min_B][1] = 2.3561944901 #135 degrees
                        tour_array[map_id_min_B][2] = 3
                        # add information about sucker: x, y, 0.2, seconds of sprayer
                        tour_array[map_id_min_B][3] = suckerCoords[0]
                        tour_array[map_id_min_B][4] = suckerCoords[1]
                        tour_array[map_id_min_B][5] = sucker_height #fix 20 cm from ground
                        tour_array[map_id_min_B][6] = tree_landmarks_data[tree_count_id][3]
                        # update map with closer point
                        map_array[map_id_min_B][1] = point_B_closer[0]
                        map_array[map_id_min_B][2] = point_B_closer[1]
                else:
                    print "No tour point has been found. New one will be added to map and tour."
                    last_id_entry_map_array = last_id_entry_map_array+1
                    map_entry = numpy.array([last_id_entry_map_array, point_A[0], point_A[1]])
                    tour_entry = numpy.array([last_id_entry_map_array, -0.7853981634, 3, suckerCoords[0], suckerCoords[1], sucker_height, tree_landmarks_data[tree_count_id][3]])
                    # add to list
                    map_entries = numpy.append(map_entries,numpy.reshape(map_entry,(1,3)),axis=0)
                    tour_entries = numpy.append(tour_entries,numpy.reshape(tour_entry,(1,7)),axis=0)
                    new_entries_flag = True
                tree_count_id = tree_count_id + 1

            # store final data
            if new_entries_flag:
                map_final = numpy.append(map_array.copy(),map_entries,axis=0)
                tour_final = numpy.append(tour_array.copy(),tour_entries,axis=0)
            else:
                map_final = map_array.copy()
                tour_final = tour_array.copy()
        
            # trasform map data to lat,lon coords
            for index in range(len(map_final)):
                lat_lon_coords = utmProj(map_final[index][1]+origin_utm_lon,map_final[index][2]+origin_utm_lat,inverse=True)
                map_final[index][1] = lat_lon_coords[1]
                map_final[index][2] = lat_lon_coords[0]

            print "New tour generated:\n", tour_final, '\n'

            # write new map file
            with open(suckers_map_file, mode='w') as output:
                output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                for row in map_final:
                    output_writer.writerow(row)

            # write new tour file
            with open(suckers_tour_file, mode='w') as output:
                output_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                tour_length = tour_final.shape(0)
                for i in range(tour_length):
                #for row in tour_final:
                    # edit: write tour starting from the bottom to the top of the array
                    row = tour_length[tour_length-1-i,:]
                    output_writer.writerow(row)

            print "Tour and Map updated.\n"
        else:
            print "\nNo sucker has been detected. No operation can be made.\n"
        
    else :
        print "Could not retrieve suckers' area"

    print "landmark_parser Parsing service called"

    return EmptyResponse()

def originCallback(originData):
    global origin_utm_lon, origin_utm_lat, origin_set
    if not origin_set :
        origin_utm_lon = originData.x
        origin_utm_lat = originData.y
        origin_set=True
        print "Origin set: ", origin_utm_lat, origin_utm_lon, '\n'

def landmark_parser():

    global threshold, landmarks, path, origin_utm_lon, origin_utm_lat, origin_set

    origin_utm_lon = 0
    origin_utm_lat = 0
    origin_set = 0

    #ROS init
    rospy.init_node('landmark_parser', anonymous=True)

    if rospy.has_param('~landmarks_topic'):
        landmarks_topic=rospy.get_param('~landmarks_topic')
    else :
        landmarks_topic=('/ekf_slam_node/visualization_marker_all_map')

    landmarks=Marker()

    rospy.Subscriber(landmarks_topic, Marker, landmarksCallback)
    rospy.Subscriber('/ekf_slam_node/origin', Point, originCallback)
    s_parsing = rospy.Service('/landmark_parser/parsing', Empty, parsingService)

    rospy.spin()

if __name__ == '__main__':
    try:
        landmark_parser()
    except rospy.ROSInterruptException:
        pass

