#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sbg_driver.msg import SbgGpsPos, SbgEkfNav
from sensor_msgs.msg import NavSatFix

global odom_slam, odom_imu, gps_slam_fix, gps_imu_fix, gps_fix, gps_imu, gps_raw

def odomSlamCallback(data):
    global odom_slam
    odom_slam=data

def odomGpsCallback(data):
    global odom_imu
    odom_imu=data

def slamFixCallback(data):
    global gps_slam_fix
    gps_slam_fix=data

def odomFixCallback(data):
    global gps_imu_fix
    gps_imu_fix=data

def gpsFixCallback(data):
    global gps_fix
    gps_fix=data

def gpsImuCallback(data):
    global gps_imu
    gps_imu=data

def gpsRawCallback(data):
    global gps_raw
    gps_raw=data

def gps_logger():

    global odom_slam, odom_imu, gps_slam_fix, gps_imu_fix, gps_fix, gps_imu, gps_raw

    #ROS init
    rospy.init_node('gps_logger', anonymous=True)

    odom_slam=Odometry()
    odom_imu=Odometry()
    gps_slam_fix=NavSatFix()
    gps_imu_fix=NavSatFix()
    gps_fix=NavSatFix()
    gps_imu=SbgEkfNav()
    gps_raw=SbgGpsPos()

    pub_odom_slam = rospy.Publisher('/logger_odom_slam', Odometry, queue_size=1)
    pub_odom_imu = rospy.Publisher('/logger_odom_imu', Odometry, queue_size=1)
    rospy.Subscriber('/ekf_slam_node/slam_odom_geo', Odometry, odomSlamCallback)
    rospy.Subscriber('/odom_gps', Odometry, odomGpsCallback)

    pub_gps_slam_fix = rospy.Publisher('/logger_gps_slam_fix', NavSatFix, queue_size=1)
    pub_gps_imu_fix = rospy.Publisher('/logger_gps_imu_fix', NavSatFix, queue_size=1)
    pub_gps_fix = rospy.Publisher('/logger_gps_fix', NavSatFix, queue_size=1)
    rospy.Subscriber('/ekf_slam_node/slam_fix', NavSatFix, slamFixCallback)
    rospy.Subscriber('/ekf_slam_node/odom_fix', NavSatFix, odomFixCallback)
    rospy.Subscriber('/fix', NavSatFix, gpsFixCallback)

    pub_gps_imu = rospy.Publisher('/logger_gps_imu', SbgEkfNav, queue_size=1)
    pub_gps_raw = rospy.Publisher('/logger_gps_raw', SbgGpsPos, queue_size=1)
    rospy.Subscriber('/ekf_nav', SbgEkfNav, gpsImuCallback)
    rospy.Subscriber('/gps_pos', SbgGpsPos, gpsRawCallback)

    #s_glob = rospy.Service('/gps_logger', GpsLogger, gpsLoggerService)
    
    #main loop
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pub_odom_slam.publish(odom_slam)
        pub_odom_imu.publish(odom_imu)
        pub_gps_slam_fix.publish(gps_slam_fix)
        pub_gps_imu_fix.publish(gps_imu_fix)
        pub_gps_fix.publish(gps_fix)
        pub_gps_imu.publish(gps_imu)
        pub_gps_raw.publish(gps_raw)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_logger()
    except rospy.ROSInterruptException:
        pass

