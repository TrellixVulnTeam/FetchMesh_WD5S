//
// Created by jacob on 1/12/21.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/PointHeadAction.h>
#include <std_msgs/String.h>

//typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> Client;
geometry_msgs::Pose2D current_pose;
ros::Publisher pub, pub2, start_segmentation_pub, point_head_pub;
void move_fetch();
void odom_cb(nav_msgs::Odometry odom);

void odom_cb(nav_msgs::Odometry odom) {
    current_pose.x = odom.pose.pose.position.x;
    current_pose.y = odom.pose.pose.position.y;
}

void move_fetch() {
    sleep(15);
    int counter = 0;
    geometry_msgs::Twist vel;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    ros::Time beginTime;
    ros::Duration dur = ros::Duration(4);
    ros::Duration stop_dur = ros::Duration(1);
    ros::Time endTime;
    std_msgs::String start_segmentation;
    start_segmentation.data = "start segmentation";
    // Path outline:
    // Start at (0,0). segment. Turn 90 degrees. Turn head. Go to (1,0). Segment
    // Go to (3,0). Turn 90 degrees. Go to (2,3) Segment. See what the mesh is looking like!
    // go right

    // Start segmentation, turn 90 degrees
    ROS_INFO("starting segmentation");
    start_segmentation_pub.publish(start_segmentation);
    ros::Duration(2).sleep();
    endTime = ros::Time::now() + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = 0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }

    // Stop rotating
    endTime = ros::Time::now() + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.angular.z = 0;
        pub.publish(vel);
    }

    // Turn head
    std_msgs::String point_head;
    point_head.data = "point_head";
    point_head_pub.publish(point_head);
    ROS_INFO("finished pointing head, resuming the trajectory");

    // go up
    while(ros::ok() && current_pose.y < 0.9) {
        ROS_INFO("I'm stuck inside this loop. heelllllp");
        vel.linear.x = 0.4;
        pub.publish(vel);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // Stop moving
    endTime = ros::Time::now() + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.linear.x = 0;
        pub.publish(vel);
    }

    // Start segmentation
    ROS_INFO("More segmentation. Letss gooooo");
    start_segmentation_pub.publish(start_segmentation);
    ros::Duration(2).sleep();

    // done with the loop!
    std_msgs::String done;
    done.data = "done";
    ROS_INFO("finished trajectory. lets hope this worked!");
    pub2.publish(done);
}

int
main (int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "fetch_mover");
    ros::NodeHandle nh;
    ROS_INFO("starting circuit around the environment...");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/odom", 5, odom_cb);

    // Create a ROS publisher for the output mesh
    pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 5);
    pub2 = nh.advertise<std_msgs::String> ("/finished_trajectory", 4);
    start_segmentation_pub = nh.advertise<std_msgs::String> ("/start_segmentation", 5);
    point_head_pub = nh.advertise<std_msgs::String> ("/start_pointing_head", 5);

    // Start the trajectory
    move_fetch();

    // Spin
    ros::spin();
}