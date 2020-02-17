#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class GetGroundTruth {
  public:
  double x;
  double y;
  double z;
  double quatx;
  double quaty;
  double quatz;
  double quatw;
  
  void Callback(const nav_msgs::Odometry::ConstPtr& msg);

};

void GetGroundTruth::Callback(const nav_msgs::Odometry::ConstPtr& msg) {
  x = double(msg->pose.pose.position.x);
  y = double(msg->pose.pose.position.y);
  z = double(msg->pose.pose.position.z);
  quatx = double(msg->pose.pose.orientation.x);
  quaty = double(msg->pose.pose.orientation.y);
  quatz = double(msg->pose.pose.orientation.z);
  quatw = double(msg->pose.pose.orientation.w);

}

int main(int argc, char** argv){

  //Calling the ros init function
  ros::init(argc, argv, "map_broadcaster");

  //Ros node handler
  ros::NodeHandle n("~");

  std::string map;
  n.param<std::string>("map", map, "/map");

  std::string odom;
  n.param<std::string>("odom", odom, "/odom_gps");

  std::string base_link;
  n.param<std::string>("base_link", base_link, "sherpa/base_link");
  
  ROS_INFO("map: %s - odom: %s - base_link: %s",map.c_str(),odom.c_str(),base_link.c_str());

  GetGroundTruth gt_sub;
  ros::Subscriber sub = n.subscribe(odom.c_str(),10, &GetGroundTruth::Callback, &gt_sub);

  //Ros rate
  ros::Rate r(100);

  //broadcaster send date to configure the map for all robots
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

    //Connect odom frame to map frame
    broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
      ros::Time::now(), map.c_str(), odom.c_str()));

    //ROS_INFO("x: %f - y: %f - z: %f - qx: %f - qy: %f - qz: %f - qw: %f",gt_sub.x, gt_sub.y, gt_sub.z,gt_sub.quatx, gt_sub.quaty, gt_sub.quatz, gt_sub.quatw);

    //Connect odom frame to base_link frame
    broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(gt_sub.quatx, gt_sub.quaty, gt_sub.quatz, gt_sub.quatw), tf::Vector3(gt_sub.x, gt_sub.y, gt_sub.z)),
      ros::Time::now(), odom.c_str(), base_link.c_str()));
 /*   
    //Connect odom frame to base_footprint frame
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(gt_sub.quatx, gt_sub.quaty, gt_sub.quatz, gt_sub.quatw), tf::Vector3(gt_sub.x, gt_sub.y, gt_sub.z)),
        ros::Time::now(),"/odom", "/base_footprint"));
        
    //Connect base_footprint frame to base_link frame
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(gt_sub.quatx, gt_sub.quaty, gt_sub.quatz, gt_sub.quatw), tf::Vector3(gt_sub.x, gt_sub.y, gt_sub.z)),
        ros::Time::now(),"/base_footprint", "sherpa/base_link"));
   */             
    r.sleep();
    
    ros::spinOnce();
    
  }
}


