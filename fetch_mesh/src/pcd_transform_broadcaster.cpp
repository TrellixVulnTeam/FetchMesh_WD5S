//
// Created by jacob on 1/29/21.
//

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

void odom_cb(nav_msgs::Odomoetry odom) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

}
int main(int argc, char** argv) {
    ros::init(arc, argv, "pcd_tf_broadcaster");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("/odom", 5, odom_cb);

}

