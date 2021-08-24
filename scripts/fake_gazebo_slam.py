#!/usr/bin/env python

import rospy
from pantheon_2d_slam.msg import navigationObstacles
from geometry_msgs.msg import Point

def gen_obstacles(obstacles):

    #init static
    obstacles.staticObstacles=[]

    #add static point 1
    temp=Point()
    temp.x=2.5
    temp.y=2.5
    temp.z=0
    obstacles.staticObstacles=obstacles.staticObstacles+[temp]
    #add static point 2
    temp=Point()
    temp.x=2.5
    temp.y=7.5
    temp.z=0
    obstacles.staticObstacles=obstacles.staticObstacles+[temp]
    #add static point 3
    temp=Point()
    temp.x=7.5
    temp.y=2.5
    temp.z=0
    obstacles.staticObstacles=obstacles.staticObstacles+[temp]
    #add static point 4
    temp=Point()
    temp.x=7.5
    temp.y=7.5
    temp.z=0
    obstacles.staticObstacles=obstacles.staticObstacles+[temp]
    #add static point 5
    temp=Point()
    temp.x=7.5
    temp.y=12.5
    temp.z=0
    obstacles.staticObstacles=obstacles.staticObstacles+[temp]
    #add static point 6
    temp=Point()
    temp.x=12.5
    temp.y=7.5
    temp.z=0
    obstacles.staticObstacles=obstacles.staticObstacles+[temp]

    #init dynamic
    obstacles.dynamicObstacles=[]

    #add dynamic point 1
    temp=Point()
    temp.x=12.5
    temp.y=12.5
    temp.z=0
    obstacles.dynamicObstacles=obstacles.dynamicObstacles+[temp]

    return obstacles

def fake_gazebo_slam():

    #ROS init
    rospy.init_node('fake_gazebo_slam', anonymous=True)

    obstacles=navigationObstacles()

    pub = rospy.Publisher('/ekf_slam_node/navigation_obstacles', navigationObstacles, queue_size=1)
    
    obstacles=gen_obstacles(obstacles)

    #main loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(obstacles)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_gazebo_slam()
    except rospy.ROSInterruptException:
        pass

