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
ros::Publisher pub, pub2;
void move_fetch();
void point_head();
void odom_cb(nav_msgs::Odometry odom);

void odom_cb(nav_msgs::Odometry odom) {
    current_pose.x = odom.pose.pose.position.x;
    current_pose.y = odom.pose.pose.position.y;
}

void point_head() {
    geometry_msgs::PointStamped goal_point;
    goal_point.point.x = 2;
    goal_point.point.y = 0;
    goal_point.point.z = 0;
//    //client.waitForServer();
//    control_msgs::PointHeadGoal goal;
//    goal.target = goal_point;
//    //client.sendGoal(goal);
//    client.waitForResult(ros::Duration(5.0));
//    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//        printf("Yay! The dishes are now clean");
//    printf("Current State: %s\n", client.getState().toString().c_str());
}

void move_fetch() {
    sleep(10);
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

    // go right
    ROS_INFO("moving the bot now...");
    ros::Rate rate(10);
    while(ros::ok() && current_pose.x < 5) {
        vel.linear.x = 0.4;
        pub.publish(vel);
        ros::spinOnce();
        rate.sleep();

    }

    // stop
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.linear.x = 0;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }

    // Turn 90 degrees to the left
    beginTime = ros::Time::now();
    endTime = beginTime + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = 0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }

    // stop
    point_head();
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.angular.z = 0;
        pub.publish(vel);
    }

    // go up
    while(ros::ok() && current_pose.y < 5) {
        vel.linear.x = 0.4;
        pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // stop
    point_head();
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.linear.x = 0;
        pub.publish(vel);
    }

    // Turn 90 degrees to the left
    beginTime = ros::Time::now();
    endTime = beginTime + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = 0.3;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }

    // stop
    point_head();
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.angular.z = 0;
        pub.publish(vel);
    }

    // go left
    while(ros::ok() && current_pose.x > 0) {
        vel.linear.x = 0.4;
        pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // stop
    point_head();
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.linear.x = 0;
        pub.publish(vel);
    }

    // Turn 90 degrees to the left
    beginTime = ros::Time::now();
    endTime = beginTime + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = 0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }

    // stop
    point_head();
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.angular.z = 0;
        pub.publish(vel);
    }

    // go down
    while(ros::ok() && current_pose.y > -2) {
        vel.linear.x = 0.4;
        pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // stop
    point_head();
    beginTime = ros::Time::now();
    endTime = beginTime + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.linear.x = 0;
        pub.publish(vel);
    }

    // done with the loop!
    std_msgs::String done;
    done.data = "done";
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
//    actionlib::SimpleActionClient<control_msgs::PointHeadAction> client("head_controller/point_head", true);
//    client.waitForServer();
    move_fetch();
    // Spin
    ros::spin();
}