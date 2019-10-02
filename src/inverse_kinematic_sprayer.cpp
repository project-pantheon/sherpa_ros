#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

geometry_msgs::Point cmd;
nav_msgs::Odometry odom;

void get_command(const geometry_msgs::Point::ConstPtr& msg) {
    cmd = *msg;
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
    
nav_msgs::Odometry sprayer_odometry(nav_msgs::Odometry Odom) {
    nav_msgs::Odometry spray_odom;
    double yaw = get_yaw(Odom);
    spray_odom = Odom;
    spray_odom.pose.pose.position.x = Odom.pose.pose.position.x + 1.0875*cos(yaw);
    spray_odom.pose.pose.position.y = Odom.pose.pose.position.y + 1.0875*sin(yaw);
    return spray_odom;
    }

double dist(double x1, double x2, double y1, double y2) {
    double dx = x1-x2;
    double dy = y1-y2;
    return sqrt( dx*dx + dy*dy );
    } 

int main(int argc, char** argv) {

    ros::init(argc, argv, "inverse_kinematic_spray");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("/pantilt_command", 10, get_command);
    ros::Subscriber odom_sub = n.subscribe("/odom", 10, get_odom);
    
    ros::Publisher pub_pan = n.advertise<std_msgs::Float64>("sherpa/pan_controller/command",10);
    ros::Publisher pub_tilt = n.advertise<std_msgs::Float64>("sherpa/tilt_controller/command", 10);
    
    double D = 0.425;
    double d = 0.512;
    
    std_msgs::Float64 pan;
    std_msgs::Float64 tilt;
    double yaw = 0.0;
    double theta = 0.0;
    double beta = 0.0; /* difference angle between yaw and theta */
    double x,y,r,R;
    nav_msgs::Odometry sprayer_odom;
    
    pan.data = 0.0;
    tilt.data = 1.56;
    cmd.z = 2.0;
    
    
    while (ros::ok()) {
        
        yaw = get_yaw(odom);
        sprayer_odom = sprayer_odometry(odom);
        x = sprayer_odom.pose.pose.position.x;
        y = sprayer_odom.pose.pose.position.y;
        
        R = dist(x,cmd.x,y, cmd.y);
        r = dist(x,cmd.x,y,y);
        theta = acos(r/R);
        beta = theta - yaw;
                
        pan.data = beta;

        tilt.data = asin((cmd.z - d)/D);
    //    ROS_INFO("z: %f,tilt: %f ", cmd.z, tilt.data);

        pub_pan.publish(pan);
        pub_tilt.publish(tilt);
        
        ros::spinOnce();
        loop_rate.sleep();
        
        }
        
    return 0;
    
    }
    
    
