#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <math.h>
#include "std_msgs/Float64.h"
#include "sherpa_ros/SprayAction.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "tf/tf.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>


typedef actionlib::SimpleActionServer<sherpa_ros::SprayAction> Server;

nav_msgs::Odometry odom;
sensor_msgs::LaserScan scan;
sensor_msgs::JointState joint_state;

void get_joint_state( const sensor_msgs::JointState::ConstPtr& msg) {
    joint_state = *msg;
    }

void get_scan( const sensor_msgs::LaserScan::ConstPtr& msg) {
    scan = *msg;
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
    
    /* da mettere forse nel client */
int scan_obstacle(sensor_msgs::LaserScan msg) {
    int obstacle_side = 0; 
    int i,j;
    double sum_sx = 0.0;
    double sum_dx = 0.0;
    
    if ( msg.header.frame_id == "front_sick") {
        for (i = 0; i < msg.ranges.size()/2; i++) {
            if (msg.ranges[i] <= 9.0) {
                sum_sx += msg.ranges[i];
                }
            }
        for (j = msg.ranges.size()/2; j < msg.ranges.size(); j++) {
            if (msg.ranges[j] <= 9.0) {
                sum_dx += msg.ranges[j];
                }
            }
    }
            
    if ( (sum_sx != 0.0) && (sum_dx != 0.0) ) obstacle_side = 3;  /* left and right obstacle */
    if ( (sum_sx != 0.0) && (sum_dx == 0.0) ) obstacle_side = 1;  /* only left obstacle */
    if ( (sum_sx == 0.0) && (sum_dx != 0.0) ) obstacle_side = 2;  /* only right obstacle */
    
    return obstacle_side;

    }
    
bool state_reached( sensor_msgs::JointState js, sherpa_ros::SprayGoal Goal) {
    if (abs(js.position[1] - Goal.goal.position.x) <= 0.1 ) {
        ROS_INFO("state reached");
        return true;
        }
    return false;
    }
    
int orientation_sign(geometry_msgs::Pose tree_pose) {
    
    double obstacle_side = scan_obstacle(scan);
    
    if ( obstacle_side == 2) return -1;
    return 1;
    }


double dist(double x1, double x2, double y1, double y2) {
    double dx = x1-x2;
    double dy = y1-y2;
    return sqrt( dx*dx + dy*dy );
    } 

int sign_yaw(double yaw) {
    double angle_tolerance = 0.1745;
    if ( (yaw >= -angle_tolerance) && (yaw <= angle_tolerance) ) return 1;
    if ( (yaw -3.14 >= -angle_tolerance) && (yaw - 3.14 <= angle_tolerance) ) return -1;
    return 1;
    }  

nav_msgs::Odometry spray_odometry(nav_msgs::Odometry Odom) {
    nav_msgs::Odometry spray_odom;
    double yaw = get_yaw(Odom);
    spray_odom = Odom;
    spray_odom.pose.pose.position.x = Odom.pose.pose.position.x + 0.8275*cos(yaw);
    spray_odom.pose.pose.position.y = Odom.pose.pose.position.y + 0.8275*sin(yaw);
    return spray_odom;
    }
    
int target_side(double x_odom, double x_tree, double yaw) {

    /* this function return 1 if the target is at the left of the robot
       -1 if the target is at the right of the robot */
    double angle_tolerance = 0.1745; /* ~10 degrees */ 
    if ( (yaw >= -angle_tolerance) && (yaw <= angle_tolerance) ) {
        if ( x_tree > x_odom ) return 1;
        if ( x_tree < x_odom ) return -1;
        }
    if ( (yaw -3.14 >= -angle_tolerance) && (yaw - 3.14 <= angle_tolerance) ) { 
        if ( x_tree < x_odom ) return 1;
        if ( x_tree > x_odom ) return -1;
        }
    return 1;
    }

/*std::string get_time() {
    auto time = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(time);
    std::string current_time = std::ctime(&end_time);
    return current_time;
    }
  */
  
std::string get_time() {
    std::time_t t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");
    auto str = oss.str();

    return str;
}
            
class SprayAction {
    
    protected: 
    
        ros::NodeHandle n;
        Server as;
        std::string action_name;
        sherpa_ros::SprayFeedback feedback;
        sherpa_ros::SprayResult result;
        
        /* Subscribers Initialization */
        ros::Subscriber odom_sub = n.subscribe("/odom", 10, get_odom);
        ros::Subscriber scan_sub = n.subscribe("/scan", 10, get_scan);
        ros::Subscriber js_sub = n.subscribe("sherpa/joint_states", 10, get_joint_state);

        
        /* Publishers Initialization */
        ros::Publisher pan_pub = n.advertise<std_msgs::Float64>("/sherpa/pan_controller/command", 10);
        ros::Publisher tilt_pub = n.advertise<std_msgs::Float64>("/sherpa/tilt_controller/command", 10);
        ros::Publisher tree_pub = n.advertise<sherpa_ros::Tree>("/tree", 10);                   
    
    public: 
    
        SprayAction(std::string name):
            as(n, name, boost::bind(&SprayAction::execute, this, _1), false), action_name(name)
            {
            as.start();
            }
            
        ~SprayAction(void)
            {
            }
        
        void execute(const sherpa_ros::SprayGoalConstPtr& goal) {
           
            bool success = false;
            int obstacle_side = scan_obstacle(scan);
            
            double yaw = get_yaw(odom);
            double beta; /* difference angle between yaw and theta */
            double x,y;
            double angle_tolerance = 0.1745; /* 10 degrees */
            nav_msgs::Odometry spray_odom;
            spray_odom = spray_odometry(odom);
            x = spray_odom.pose.pose.position.x;
            y = spray_odom.pose.pose.position.y;
            
            sherpa_ros::Tree processed_tree;
            
            double R = 0.0;
            double d = 0.0;
            double theta = 0.0;
            int side_sign = target_side(x, goal->goal.position.x, yaw);
            R = dist(x,goal->goal.position.x,y, goal->goal.position.y);
            d = dist(x,goal->goal.position.x,y,y);
            theta = acos(d/R);
            beta = ( side_sign*theta - yaw ); /* this will be the pan angle */
            
            if(goal->goal.sprayer.task_status >= 2) as.setSucceeded(); /* if target has been sprayed 2 times, skip */            
            
            std_msgs::Float64 pan;
            std_msgs::Float64 tilt;
            pan.data = beta;
            tilt.data = -1.57;
            
            /* updating tree info */
            processed_tree.position.x = x;
            processed_tree.position.y = y;
            processed_tree.sprayer.time = get_time();
            processed_tree.sprayer.task_status += 1;
            processed_tree.sprayer.task_info = "Some info";
            tree_pub.publish(processed_tree);   
            
            if (state_reached) ROS_INFO("ok");         
            
            pan_pub.publish(pan);
            tilt_pub.publish(tilt);
                       
            /* return to initial position */
            sleep(5);
            pan.data = 0.0;
            tilt.data = 0.0;
            pan_pub.publish(pan);
            tilt_pub.publish(tilt);
            success = true;    
            sleep(5);                           
                        
            if (success) as.setSucceeded();
            }
    };                     
        
        
int main(int argc, char** argv) {
    ros::init(argc, argv, "spray_target_server");
    SprayAction sprayer("spray_target_server");   
    ros::spin();
    return 0;
    }

