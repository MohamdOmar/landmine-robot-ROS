#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <iostream>

using namespace std;

double metal_link_x = 0;
double metal_link_y = 0;
double metal_link_z = 0;

geometry_msgs::Twist twist;

void twistCB(const geometry_msgs::Twist::ConstPtr &twist_msg){
  twist.linear.x = (*twist_msg).linear.x;
}

unsigned int x = 0;
int cnt = 0;



int main(int argc, char** argv){
  ros::init(argc, argv, "marker");

  ros::NodeHandle node;
  //ros::Publisher velocity_pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
  //ros::Subscriber mine_subscriber = node.subscribe("/mineMessage",1000,minecallback);
  ros::Subscriber mine_subscriber = node.subscribe("/mineMessage", 1000, twistCB);
  ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  tf::TransformListener frame_listener;

  ros::Rate rate(10);

  while (node.ok()){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = cnt++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.lifetime = ros::Duration();
    ros::spinOnce();

    tf::StampedTransform my_link;

    try{
      frame_listener.waitForTransform("/odom", "/metal_detector", ros::Time(0), ros::Duration(3.0));
      frame_listener.lookupTransform("/odom", "/metal_detector", ros::Time(0), my_link);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

  metal_link_x = my_link.getOrigin().x();
  metal_link_y = my_link.getOrigin().y();
  metal_link_z = my_link.getOrigin().z();

  ROS_INFO_STREAM("metal_detector_pose_x: " << metal_link_x);
  ROS_INFO_STREAM("metal_detector_pose_y: " << metal_link_y);
  ROS_INFO_STREAM("metal_detector_pose_z: " << metal_link_z);

  /*cout <<"enter marker id:";
  cin >> x;
  cout << endl;*/

 //if(twist.linear.x == 1){     
    marker.pose.position.x = metal_link_x;
    marker.pose.position.y = metal_link_y;
    marker.pose.position.z = metal_link_z;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 10.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker_pub.publish(marker); //}    

 if(twist.linear.x == 2){     
    marker.pose.position.x = metal_link_x;
    marker.pose.position.y = metal_link_y;
    marker.pose.position.z = metal_link_z;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker_pub.publish(marker);}
    rate.sleep();
  }
  
  return 0;
}
