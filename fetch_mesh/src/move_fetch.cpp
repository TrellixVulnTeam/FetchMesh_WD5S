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
#include <tf/transform_listener.h>
tf::TransformListener *tf_listener = NULL;

//typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> Client;
geometry_msgs::PoseStamped current_pose;
ros::Publisher pub, pub2, start_segmentation_pub, point_head_pub, start_reconstruction_pub;
void move_fetch();
void simple_world_trajectory();
void odom_cb(nav_msgs::Odometry odom);
void stop();
void turn90_ccw();
void turn90_cw();
void move_forward_to_point(double threshold, char axis);

void odom_cb(nav_msgs::Odometry odom) {
    geometry_msgs::PoseStamped new_pose;
    current_pose.header.frame_id = "/odom";
    current_pose.pose = odom.pose.pose;
    tf_listener->transformPose("/map", ros::Time(0), current_pose, "/odom", new_pose);
    current_pose = new_pose;
}

void stop() {
    geometry_msgs::Twist vel;
    ros::Duration stop_dur = ros::Duration(1);
    ros::Time endTime;
    endTime = ros::Time::now() + stop_dur;
    while(ros::Time::now() < endTime) {
        vel.angular.z = 0;
        vel.linear.x = 0;
        pub.publish(vel);
    }
}

// Make this much more exact, need to get the /odom for the rotation somehow
void turn90_ccw() {
    geometry_msgs::Twist vel;
    ros::Time endTime;
    ros::Duration dur = ros::Duration(4);
    endTime = ros::Time::now() + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = 0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }
    stop();
}

void turn90_cw() {
    geometry_msgs::Twist vel;
    ros::Time endTime;
    ros::Duration dur = ros::Duration(4);
    endTime = ros::Time::now() + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = -0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }
    stop();
}

void turn45_cw() {
    geometry_msgs::Twist vel;
    ros::Time endTime;
    ros::Duration dur = ros::Duration(2);
    endTime = ros::Time::now() + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = -0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }
}

// axis must be 'x' or 'y' at this point
// move forward until at the threshold along the chosen axis
// have to adjust for cases when robot is above or below the threshold
void move_forward_to_point(double threshold, char axis) {
    geometry_msgs::Twist vel;
    if (axis == 'x') {
        if (threshold > current_pose.pose.position.x) {
            while(ros::ok() && current_pose.pose.position.x < threshold) {
                vel.linear.x = 0.4;
                pub.publish(vel);
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
        }

        else if (threshold < current_pose.pose.position.x) {
            while(ros::ok() && current_pose.pose.position.x > threshold) {
                vel.linear.x = 0.4;
                pub.publish(vel);
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
        }
    }

    else if (axis == 'y') {
        if (threshold > current_pose.pose.position.y) {
            while(ros::ok() && current_pose.pose.position.y < threshold) {
                vel.linear.x = 0.4;
                pub.publish(vel);
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
        }

        else if (threshold < current_pose.pose.position.y) {
            while(ros::ok() && current_pose.pose.position.y > threshold) {
                vel.linear.x = 0.4;
                pub.publish(vel);
                ros::spinOnce();
                //ros::Duration(0.1).sleep();
            }
        }
    }

    stop();
}

// Spins fetch slowly 360 degrees to generate a map of the surroundings
void full_circle_map() {
    geometry_msgs::Twist vel;
    ros::Time endTime;
    ros::Duration dur = ros::Duration(16);
    endTime = ros::Time::now() + dur;
    while(ros::Time::now() < endTime ) {
        vel.angular.z = 0.6;
        pub.publish(vel);
        ros::Duration(0.1).sleep();
    }
    stop();
}

// turn 360 degrees, stop to after rotating 45 degrees
void rseg_and_greedy_circle_trajectory() {
    sleep(35);
    ROS_INFO("starting the trajectory");
    std_msgs::String start, finish;
    start.data = "start segmentation";
    finish.data = "export the mesh buddy";

    // trajectory
    for (int i=0; i<9; i++) {
        ROS_INFO("Starting rotation %d", i + 1);
        if (i>4) {  // have to wait a bit for the monte carlo localization to start becoming accurate
            start_segmentation_pub.publish(start);
            sleep(2);
        }
        turn45_cw();
    }
    sleep(5);
    start_reconstruction_pub.publish(finish);
}

void greedy_rseg_simple_world_test_trajectory() {
    sleep(35);
    ROS_INFO("Starting the trajectory");
    std_msgs::String start, finish;
    start.data = "start segmentation";
    finish.data = "export the mesh buddy";

    start_segmentation_pub.publish(start);
    sleep(5);
    start_reconstruction_pub.publish(finish);
}

// A trajectory to be used along with the greedy_triangulation script
void greedy_triangulation_circle_trajectory() {
    sleep(35);
    ROS_INFO("starting the trajectory");
    std_msgs::String start, finish;
    start.data = "start segmentation";
    finish.data = "export the mesh buddy";

    for (int i=0; i<9; i++) {
        ROS_INFO("Sending scan %d to mesh reconstruction script", i+1);
        if (i>4) {  // have to wait a bit for the monte carlo localization to start becoming accurate
            start_reconstruction_pub.publish(start);
            sleep(2);
        }
        turn45_cw();
    }
    sleep(5);
    start_reconstruction_pub.publish(finish);
}

void simple_world_trajectory() {
    sleep(35);
    ROS_INFO("starting the trajectory");
    std_msgs::String start_segmentation;
    start_segmentation.data = "start segmentation";

    // Turn head 90 degrees
    std_msgs::String point_head;
    point_head.data = "point_head";
    point_head_pub.publish(point_head);
    sleep(10);
    ROS_INFO("finished pointing head, starting the trajectory");

    // Start segmentation
    turn90_ccw();
    start_segmentation_pub.publish(start_segmentation);
    sleep(2);

    // trajectory
    move_forward_to_point(2.5, 'y');
    turn90_cw();
    move_forward_to_point(1.5, 'x');
    start_segmentation_pub.publish(start_segmentation);
    sleep(2);
    move_forward_to_point(4, 'x');
    turn90_cw();
    move_forward_to_point(0, 'y');
    start_segmentation_pub.publish(start_segmentation);
    sleep(2);
    move_forward_to_point(-2.5, 'y');
    turn90_cw();
    move_forward_to_point(1.5, 'x');
    start_segmentation_pub.publish(start_segmentation);
    sleep(2);
    return;
}
void move_fetch() {
    sleep(15);
    std_msgs::String start_segmentation;
    start_segmentation.data = "start segmentation";

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // turn 90 degrees counterclockwise
    turn90_ccw();

    // Turn head 90 degrees
    std_msgs::String point_head;
    point_head.data = "point_head";
    point_head_pub.publish(point_head);
    sleep(10);
    ROS_INFO("finished pointing head, starting the trajectory");

    // head up to (-1, 1)
    move_forward_to_point(1, 'y');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // head up to (-1,2.5)
    move_forward_to_point(2.5, 'y');

    // turn 90 cw
    turn90_cw();

    // head over to (2, 2.5)
    move_forward_to_point(2, 'x');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // head over to (4, 1)
    move_forward_to_point(4, 'x');
    turn90_cw();
    move_forward_to_point(1, 'y');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // head to (4, 0)
    move_forward_to_point(0, 'y');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // head to (4, -1)
    move_forward_to_point(-1, 'y');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // head to (2, -2.5)
    move_forward_to_point(-2.5, 'y');
    turn90_cw();
    move_forward_to_point(2, 'x');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);

    // head to (0, -1)
    move_forward_to_point(0, 'x');
    turn90_cw();
    move_forward_to_point(-1, 'y');

    // Start segmentation
    start_segmentation_pub.publish(start_segmentation);
    sleep(10);
}

int
main (int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "fetch_mover");
    ros::NodeHandle nh;
    ROS_INFO("starting circuit around the environment...");
    tf_listener = new (tf::TransformListener);

    // Set up odom subscribers and various controller publishers
    ros::Subscriber sub = nh.subscribe ("/odom", 5, odom_cb);
    pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 5);
    pub2 = nh.advertise<std_msgs::String> ("/finished_trajectory", 4);
    start_segmentation_pub = nh.advertise<std_msgs::String> ("/start_segmentation", 5);
    point_head_pub = nh.advertise<std_msgs::String> ("/start_pointing_head", 5);
    start_reconstruction_pub = nh.advertise<std_msgs::String> ("/start_reconstruction", 5);

    // Start the trajectory
    greedy_rseg_simple_world_test_trajectory();

    // Spin
    ros::spin();
}