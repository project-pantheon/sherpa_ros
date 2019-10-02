#include <ros/ros.h> 

#include "std_msgs/Float64.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/PoseArray.h"

/*

class Get_Joint_State {
    public:
    double pan;
    double tilt;
    
    void PanSubCallback(const std_msgs::Float64::ConstPtr& msg);
    void TiltSubCallback(const std_msgs::Float64::ConstPtr& msg);
};

void Get_Joint_State::PanSubCallback(const std_msgs::Float64::ConstPtr& msg) {
    pan = msg->data;
}

void Get_Joint_State::TiltSubCallback(const std_msgs::Float64::ConstPtr& msg) {
    tilt = msg->data;
}

*/

class PanTilt {

    public:
    double pan;
    double tilt;
    
    void pantilt_command_callback(const geometry_msgs::Point::ConstPtr& msg);
};

void PanTilt::pantilt_command_callback(const geometry_msgs::Point::ConstPtr& msg) {
    pan = msg->x;
    tilt = msg->y;
    }


int main(int argc, char** argv) {
    ros::init(argc, argv, "spray_target_client");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    PanTilt pntlt;
    ros::Subscriber sub1 = n.subscribe("/test_pantitl", 10, &PanTilt::pantilt_command_callback, &pntlt);
    
    /*
    Get_Joint_State joint_state;    
    ros::Subscriber sub1 = n.subscribe("sherpa/pan_controller/command", 10, &Get_Joint_State::PanSubCallback, &joint_state);
    ros::Subscriber sub2 = n.subscribe("sherpa/tilt_controller/command", 10, &Get_Joint_State::TiltSubCallback, &joint_state);
    
    */
    
    ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/pantilt/pantilt_position_joint_trajectory_controller/command",10);
    
    trajectory_msgs::JointTrajectory joint_traj;
   // trajectory_msgs::JointTrajectoryPoint points;
    joint_traj.joint_names.resize(2);
    joint_traj.points.resize(1);
    joint_traj.points[0].positions.resize(2);
    

    pntlt.pan = 0.0;
    pntlt.tilt = 0.0;
    

    while(ros::ok()) {

      // joint_traj.header.stamp = ros::Time::now();
     //  joint_traj.header.frame_id = "pantilt test";        

       joint_traj.joint_names[0] = "rbsherpa_hl_torso_pantilt_pan_joint";
       joint_traj.joint_names[1] = "rbsherpa_hl_torso_pantilt_tilt_joint";  
       joint_traj.points[0].time_from_start = ros::Duration(1);     

       joint_traj.points[0].positions[0] = pntlt.pan;
       joint_traj.points[0].positions[1] = pntlt.tilt;

       pub.publish(joint_traj);
           
       ros::spinOnce();
       loop_rate.sleep();
       
       }
       
       return 0;
}
