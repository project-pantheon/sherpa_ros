#include <ros/ros.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include "sherpa_ros/SprayAction.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "tf/tf.h"



/* Declaring a new Move base Action Client */ 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef move_base_msgs::MoveBaseGoal MBG;
typedef geometry_msgs::PoseStamped PS;
typedef nav_msgs::Path PH;


/* Declaring a new Spray Action Client */
typedef actionlib::SimpleActionClient<sherpa_ros::SprayAction> SprayActionClient;


/* Global Variables */    
MBG goal;
PH path;
geometry_msgs::PoseArray trees;
sherpa_ros::SprayGoal sprayer_goal;
nav_msgs::Odometry odom;

void get_path(const nav_msgs::Path::ConstPtr& msg) {
    path = *msg;
    }


void get_goal(PH path, int i) {
     
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now();
     goal.target_pose.pose = path.poses[i].pose;
                
    }
    
void get_trees(const geometry_msgs::PoseArray::ConstPtr& msg) {
    
    trees = *msg;
    
    }
    
void get_sprayer_goal(geometry_msgs::PoseArray Trees, int i) {
    
    sprayer_goal.goal.position.x = Trees.poses[i].position.x;
    sprayer_goal.goal.position.y = Trees.poses[i].position.y;
    sprayer_goal.goal.position.z = 0;
    
    }

        
void get_odom( const nav_msgs::Odometry::ConstPtr& msg) {
    
    odom = *msg;

    }
    
double get_yaw(nav_msgs::Odometry Odom) {
    tf::Quaternion q(
        Odom.pose.pose.orientation.x,
        Odom.pose.pose.orientation.y,
        Odom.pose.pose.orientation.z,
        Odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
    }
    
    
bool correct_yaw(double yaw) {
    double angle_tolerance = 0.1745;
    if ( (yaw >= -angle_tolerance) && (yaw <= angle_tolerance) ) return true;
    if ( (yaw -3.14 >= -angle_tolerance) && (yaw - 3.14 <= angle_tolerance) ) return true;
    return false;
    }    
    
int main(int argc, char** argv) {

    ros::init(argc, argv, "navigation_goals");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    /* Init move_base client */ 
    MoveBaseClient ac("move_base", true);
    SprayActionClient acs("spray_target_server", true);
    
    /* Waiting for servers to start */ 
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server");
    }
    
    while(!ac.waitForServer(ros::Duration(5.0))) 
        ROS_INFO("Waiting for the spray_target action server");
        
            
 
    ros::Subscriber sub = n.subscribe("/path_sherpa",10, get_path);
    ros::Subscriber sub_trees = n.subscribe("/target_trees", 10, get_trees);
    ros::Subscriber odom_sub = n.subscribe("/odom", 10, get_odom);    
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/topic_prova", 10);
    
    int size = 0;
    int size_trees = 0;
    double yaw;
    
    while(ros::ok()) {
 
        
        
        //pub.publish(orientation);
        size = path.poses.size();
        size_trees = trees.poses.size();
        if (size > 0 ) {
            for (int i = 0; i <= size; i++) {
                
                pub.publish(odom);
                yaw = get_yaw(odom);
                ROS_INFO("%f - x: %f - y: %f", yaw, odom.pose.pose.position.x, odom.pose.pose.position.y);
                get_goal(path, i);
                if ( correct_yaw(yaw) == true ) {
                     get_sprayer_goal(trees,i);
                     acs.sendGoal(sprayer_goal);
                     acs.waitForResult();
                     }
                ac.sendGoal(goal);
                ac.waitForResult();
                
                
                }
            }
            
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    return 0;
    
}
    

        
        
