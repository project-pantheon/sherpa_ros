#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv){

    //Calling the ros init function
    ros::init(argc, argv, "camera_gui_manager");

    //Ros node handler
    ros::NodeHandle n;

    //Ros rate
    ros::Rate r(100);

    //broadcaster sends data
    tf::TransformBroadcaster broadcaster;
    
    //listener asks data    
    tf::TransformListener listener;

    geometry_msgs::Point position;
    position.x=0;
    position.y=0;
    position.z=15;

    //90 degress on pitch to turn the camera down
    geometry_msgs::Quaternion orientation;
    orientation.x=0;
    orientation.y=0.707106781186548;
    orientation.z=0;
    orientation.w=0.707106781186548;
    

    ros::Publisher camera_gui_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

    gazebo_msgs::ModelState camera_state;

    while(n.ok()){

        tf::StampedTransform transform;
        try{
            listener.lookupTransform("map", "sherpa/base_link",  
                                   ros::Time(0), transform);
            ROS_INFO("ok");
        }
        catch (tf::TransformException ex){
            ROS_ERROR("camera_gui_manager: %s - it appears only one time",ex.what());
            ros::Duration(1.0).sleep();
        }

        position.x = transform.getOrigin().x();
        position.y = transform.getOrigin().y();

        camera_state.model_name="camera_gui"; //name of the MODEL on gazebo 
        camera_state.pose.position=position;
        camera_state.pose.orientation=orientation;
        camera_state.reference_frame="map"; //reference FRAME

        camera_gui_pub.publish(camera_state);

        // send position
        broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w), tf::Vector3(position.x, position.y, position.z)),
        ros::Time::now(),"/map", "/camera_base_link"));
        r.sleep();
    }
}
