#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

/* 
 *Class initialization 
 * this class provides to memorize linear_x and linear_y velocity 
 * and steering angle for the four robot wheels
 */
 
class SubToOmniCmd {
  public: 
  double linX;
  double linY;
  double omega;
  
  void OmniSubCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

/* Subscriber Callback */
void SubToOmniCmd::OmniSubCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  linX = double(msg->linear.x);
  linY = double(msg->linear.y);
  omega = double(msg->angular.z);
}


int sign(double num) {
  if (num <0.0) return -1;
  return 1;
}

double wrap(double value)
{  // wrap the angle between -M_PI/2 and M_PI/2
  double eps = 1e-5;
  while (value > 0.5 * M_PI + eps)
    value -= M_PI;
  while (value < -0.5 * M_PI - eps)
    value += M_PI;
  return value;
}



int main(int argc, char** argv) {
  
  ros::init(argc, argv, "omni_cmd");
  ros::NodeHandle omni;
  
  /* Subcriber declaration */ 
  SubToOmniCmd sub_omni;
  ros::Subscriber sub = omni.subscribe("sherpa/omni_cmd", 10, &SubToOmniCmd::OmniSubCallback, &sub_omni);
  
  ros::Publisher left_front_steer_pub = omni.advertise<std_msgs::Float64>("sherpa/left_front_steering_controller/command", 10);
  ros::Publisher right_front_steer_pub = omni.advertise<std_msgs::Float64>("sherpa/right_front_steering_controller/command", 10);
  ros::Publisher left_rear_steer_pub = omni.advertise<std_msgs::Float64>("sherpa/left_rear_steering_controller/command", 10);
  ros::Publisher right_rear_steer_pub = omni.advertise<std_msgs::Float64>("sherpa/right_rear_steering_controller/command", 10);
  
  ros::Publisher left_front_axle_pub = omni.advertise<std_msgs::Float64>("sherpa/left_front_axle_controller/command", 10);
  ros::Publisher right_front_axle_pub = omni.advertise<std_msgs::Float64>("sherpa/right_front_axle_controller/command", 10);
  ros::Publisher left_rear_axle_pub = omni.advertise<std_msgs::Float64>("sherpa/left_rear_axle_controller/command", 10);
  ros::Publisher right_rear_axle_pub = omni.advertise<std_msgs::Float64>("sherpa/right_rear_axle_controller/command", 10);
  
  double h = 1.335;     /* wheelbase */ 
  double w = 0.645;     /* track */ 
  double R;
  double radius = 0.16; /* wheel radius */ 
  
  double velocity;
  double vx;
  double vy; 
  double omega;  
  double alpha;
  double beta;
  double phi;

  double wx[3];
  double wy[3];
  double v[3];
  double a[3];
  double r[3];
  double x1;
  double y1;
  
  int const Pi = 3.1416;
  
  std_msgs::Float64 lfs; /* left front steering angle */
  std_msgs::Float64 rfs; /* right front steering angle */
  std_msgs::Float64 lrs; /* left rear steering angle */
  std_msgs::Float64 rrs; /* right rear steering angle */
  
  std_msgs::Float64 lfa; /* left front axle velocity */
  std_msgs::Float64 rfa; /* right front axle velocity */
  std_msgs::Float64 lra; /* left rear axle velocity */
  std_msgs::Float64 rra; /* right rear axle velocity */
  
  ros::Rate loop_rate(10);
  
  sub_omni.linX = 0.0;
  sub_omni.linY = 0.0;
  sub_omni.omega = 0.0;
  
  while(ros::ok()) {
    
    vx = sub_omni.linX;
    vy = sub_omni.linY;
    omega = sub_omni.omega;
    
    
   if ( (omega == 0.0) && ( vx == 0.0) && (vy == 0.0) ) {
         
         /* Stop condition */   
            phi = 0.0;
            
            lfs.data = phi;
            rfs.data = phi;
            lrs.data = phi;
            rrs.data = phi;
            
            lfa.data = 0.0;
            rfa.data = 0.0;
            lra.data = 0.0;
            rra.data = 0.0;
            
            }    
              
   else {
   
        /* Roto-traslation */ 
       
            x1 = h/2;
            y1 = w/2;
            
            /* Left front wheel: index 0 */
            wx[0] = vx - omega * y1;
            wy[0] = vy + omega * x1;
            v[0] = sign(wx[0]) * sqrt(wx[0]*wx[0] + wy[0]*wy[0]);
            v[0] = v[0] / radius;
            a[0] = wrap(atan2(wy[0], wx[0]));

            /* Right front wheel: index 1 */
            wx[1] = vx + omega * y1;
            wy[1] = vy + omega * x1;
            v[1] = sign(wx[1]) * sqrt(wx[1]*wx[1] + wy[1]*wy[1]);
            v[1] = v[1] / radius;
            a[1] = wrap(atan2(wy[1], wx[1]));

            /* Left rear wheel: index 2 */
            wx[2] = vx - omega * y1;
            wy[2] = vy - omega * x1;
            v[2] = sign(wx[2]) * sqrt(wx[2]*wx[2] + wy[2]*wy[2]);
            v[2] = v[2] / radius;
            a[2] = wrap(atan2(wy[2], wx[2]));
            
             /* right rear wheel: index 3 */
            wx[3] = vx + omega * y1;
            wy[3] = vy - omega * x1;
            v[3] = sign(wx[3]) * sqrt(wx[3]*wx[3] + wy[3]*wy[3]);
            v[3] = v[3] / radius;
            a[3] = wrap(atan2(wy[3], wx[3]));     
            
            lfs.data = a[0];
            rfs.data = a[1];
            lrs.data = a[2];
            rrs.data = a[3];
            
            lfa.data = v[0];
            rfa.data = v[1];
            lra.data = v[2];
            rra.data = v[3];                  
      }
           
      
    left_front_steer_pub.publish(lfs);
    right_front_steer_pub.publish(rfs);
    left_rear_steer_pub.publish(lrs);
    right_rear_steer_pub.publish(rrs);
            
    left_front_axle_pub.publish(lfa);
    right_front_axle_pub.publish(rfa);
    left_rear_axle_pub.publish(lra);
    right_rear_axle_pub.publish(rra);   

    
    ros::spinOnce();
    loop_rate.sleep();
    }
    

    
  return 0;
}
    

    
    
      
      
  
  
