#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "nav_msgs/Odometry.h"



/* Class initialization
 * this class provides to memorize velocity and steering_angle published on the 
 * akrm_cmd topic 
 */
class SubToAckermannCmd {
  public:
  double velocity;
  double steering_angle;
  
  
  void AckermannSubCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

/* Subscriber Callback */
void SubToAckermannCmd::AckermannSubCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  velocity = double(msg->linear.x);
  steering_angle = double(msg->angular.z);
}

/* Sign function */
int sign(double num) {
  if (num < 0.0) return -1;
  return 1;
}


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "akrm_cmd");
  ros::NodeHandle akrm;
  //ros::Publisher value_pub = akrm.advertise<geometry_msgs::Twist>("sherpa/akrm_cmd", 10);
  SubToAckermannCmd sub_akrm;
  ros::Subscriber vel_gamma = akrm.subscribe("/base/base_pad/cmd_vel", 10, &SubToAckermannCmd::AckermannSubCallback, &sub_akrm);
  
  ros::Publisher left_front_steer_pub = akrm.advertise<std_msgs::Float64>("left_front_steering_controller/command", 10);
  ros::Publisher right_front_steer_pub = akrm.advertise<std_msgs::Float64>("right_front_steering_controller/command", 10);
  ros::Publisher left_rear_steer_pub = akrm.advertise<std_msgs::Float64>("left_rear_steering_controller/command", 10);
  ros::Publisher right_rear_steer_pub = akrm.advertise<std_msgs::Float64>("right_rear_steering_controller/command", 10);
 

  ros::Publisher left_front_axle_pub = akrm.advertise<std_msgs::Float64>("left_front_axle_controller/command", 10);
  ros::Publisher right_front_axle_pub = akrm.advertise<std_msgs::Float64>("right_front_axle_controller/command", 10);
  ros::Publisher left_rear_axle_pub = akrm.advertise<std_msgs::Float64>("left_rear_axle_controller/command", 10);
  ros::Publisher right_rear_axle_pub = akrm.advertise<std_msgs::Float64>("right_rear_axle_controller/command", 10);





  double h = 1.335;
  double w = 0.855;
  
  double gamma;
  double R;           /* Curvature radius middle rear axel  */
  double R_lf;        /* Curvature radius left front wheel  */ 
  double R_rf;        /* Curvature radius right front wheel */
  double R_lr;        /* Curvature radius left rear wheel   */
  double R_rr;        /* Curvature radius right rear wheel  */
  double omega;       /* Angular velocity rigid body        */

  int count;
  double gamma_sx, gamma_dx;
  
  /* ros message initialization */
  std_msgs::Float64 lf_gamma;
  std_msgs::Float64 rf_gamma;
  std_msgs::Float64 lr_gamma;
  std_msgs::Float64 rr_gamma;

  std_msgs::Float64 input_velocity;
  std_msgs::Float64 lf_vel;           /* left front wheel velocity   */
  std_msgs::Float64 rf_vel;           /* right front wheel velocity  */
  std_msgs::Float64 lr_vel;           /* left rear wheel velocity    */
  std_msgs::Float64 rr_vel;           /* right rear wheel velocity   */
  
  
  sub_akrm.velocity = 0.0;
  sub_akrm.steering_angle = 0.0;
  
  ros::Rate loop_rate(10);
  
  /* Loop */
  while(ros::ok()) {
    
    
    /* Ackermann kinematic */
    gamma = sub_akrm.steering_angle;
    R = h/tan(fabs(gamma));
    
  
    gamma_sx = atan2(h,(R - sign(gamma)*h/2))*sign(gamma);
    gamma_dx = atan2(h,(R + sign(gamma)*h/2))*sign(gamma);

    
    lf_gamma.data = gamma_sx;
    rf_gamma.data = gamma_dx;
    lr_gamma.data = 0;
    rr_gamma.data = 0;
    
    
    input_velocity.data = sub_akrm.velocity;
    omega = input_velocity.data/R;
    
    if (gamma == 0.0) {
     
    lf_vel.data = input_velocity.data/0.16;
    rf_vel.data = input_velocity.data/0.16;
    lr_vel.data = input_velocity.data/0.16;
    rr_vel.data = input_velocity.data/0.16;

    
    }else{
    /* Computing curvature radius for each wheel */
    R_lr = R - sign(gamma)*w/2;
    R_rr = R + sign(gamma)*w/2;
    R_lf = sqrt(h*h + R_lr*R_lr);
    R_rf = sqrt(h*h + R_rr*R_rr);
    
    /* Computing linear velocity for each wheel */
    lf_vel.data = omega*R_lf;
    rf_vel.data = omega*R_rf;
    lr_vel.data = omega*R_lr;
    rr_vel.data = omega*R_rr; 
    
    lf_vel.data = lf_vel.data/0.16;
    rf_vel.data = rf_vel.data/0.16;
    lr_vel.data = lr_vel.data/0.16;
    rr_vel.data = rr_vel.data/0.16;

    }
    
    /* Publishing data for the steering_angle */
    left_front_steer_pub.publish(lf_gamma);
    right_front_steer_pub.publish(rf_gamma);
    left_rear_steer_pub.publish(lr_gamma);
    right_rear_steer_pub.publish(rr_gamma);
    
    /* Publishing data for the axle velocity */
    left_front_axle_pub.publish(lf_vel);
    right_front_axle_pub.publish(rf_vel);
    left_rear_axle_pub.publish(lr_vel);
    right_rear_axle_pub.publish(rr_vel);
    
    
    ros::spinOnce();
    loop_rate.sleep();
    count++;
    }
    

    
  return 0;
}
