#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double const PI = 3.14159265359;

/* Class GetOdom lets subscribe to nav_msgs/Odometry type topic */ 
class GetOdom {
  public:
  double x;
  double y;
  double orientation;
  
  void GetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

/* Way Point Callback */
void GetOdom::GetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  x = double(msg->pose.pose.position.x);
  y = double(msg->pose.pose.position.y);
  
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    orientation = yaw;

}

/* Fake class used for testing singularity. The purpose of this class is to obtain a structure of nav_msgs/Odometry type that, unlike the previous structure, allows to insert as input an orientation as a yaw angle instead of a quaternion */
class FakeOdom {
  public:
  double x;
  double y;
  double orientation;
  
  void FakeOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

/* Way Point Callback */
void FakeOdom::FakeOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  x = double(msg->pose.pose.position.x);
  y = double(msg->pose.pose.position.y);
  orientation = double(msg->pose.pose.orientation.z);

}


/* Sign function */
int sign(double num) {
  if (num < 0.0) return -1;
  return 1;
}


int main(int argc, char** argv) {
  
  /* Odometry variables */
  double wp_x;
  double wp_y;              /* way point coordinates and orientation */
  double wp_orientation;
  
  double rb_x;
  double rb_y;              /* robot coordinates and orientation */
  double rb_orientation;
  
  double dx;
  double dy;

  double l = 1.335;

  /* Control variables */
  double rho;
  double gamma_;
  double delta; 
  double theta;
  
  double vel;
  double omega;
  double phi;

  /* Gain */
  double k1 = 1.0*0.7;
  double k2 = 2.0*0.7;
  double k3 = 5.0*0.7;
  
  ros::init(argc, argv, "way_point");
  ros::NodeHandle wp;
  
  ros::Publisher pubber = wp.advertise<nav_msgs::Odometry>("sherpa/way_point",10);
  FakeOdom wp_sub;
  ros::Subscriber subscriber = wp.subscribe("sherpa/way_point", 10, &FakeOdom::FakeOdomCallback, &wp_sub);
  GetOdom odom_sub;
  ros::Subscriber subscriber2 = wp.subscribe("/odom", 10, &GetOdom::GetOdomCallback, &odom_sub);
  ros::Publisher publisher = wp.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  
  /* Message published on akrm_cmd topic */
  geometry_msgs::Twist akrm_cmd_pub;
  
  ros::Rate loop_rate(10);
  
  /* Loop */
  while(ros::ok()) {
    
    /* Variables initialization */
    wp_x = wp_sub.x;
    wp_y = wp_sub.y;
    wp_orientation = wp_sub.orientation;
    ROS_INFO("Goal orientation: %f", wp_orientation);
    rb_x = odom_sub.x;
    rb_y = odom_sub.y;
    rb_orientation = odom_sub.orientation;
    ROS_INFO("Robot orientation: %f", rb_orientation);
   
    dx = wp_x - rb_x;
    dy = wp_y - rb_y;
    
    rho = sqrt(dx*dx + dy*dy);  /* Distance to goal */
    theta = rb_orientation;
    gamma_ = atan2(dy, dx) - theta;
    delta = gamma_ + theta - wp_orientation;
    
    /* Computing linear and angular velocity */   
    vel = k1*(rho*sign(cos(gamma_)));
    omega = k2*gamma_+k1*(sin(gamma_)*sign(cos(gamma_))/gamma_)*(gamma_+k3*delta);
    omega = omega*(l/vel);
    ROS_INFO("Angular velocity omega: %f", omega);
    ROS_INFO("Robot Velocity': %f", vel);
    
    /* Computing steering angle for ackermann kinematic*/
    phi = acos(1/(sqrt(omega*omega +1)))*sign(omega);
    ROS_INFO("Steering angle: %f", phi);
    
    akrm_cmd_pub.linear.x = vel*0.16;
    akrm_cmd_pub.angular.z = phi;
    ROS_INFO("Published steering_angle angle: %f",akrm_cmd_pub.angular.z); 
    
   
    
    /* Publising data on akrm_cmd topic*/ 
    publisher.publish(akrm_cmd_pub);
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }
 
 return 0; 
}
  
